#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_proc.h"
#include <cassert>
#include <iostream>
#include <iomanip> 
#include <algorithm>
#include <vector>
#include <list>

FILE* FP;               // File handler
int op_type, dest, src1, src2;  // Variables are read from trace file
unsigned long int pc; // Variable holds the pc read from input file
proc_params params;
char* trace_file;
unsigned long int exec_stage_size = 0;
////////////////////////////    FUNCTION DECLARATIONS    ////////////////////////////
void Retire();
void Writeback();
void Execute();
void Issue();
void Dispatch();
void RegRead();
void Rename();
void Decode();
void Fetch(int);
bool Advance_Cycle();
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////    OBJECTS   ////////////////////////////
unsigned int DE_bundlesize = 0;
unsigned int DI_bundlesize = 0;
unsigned int RR_bundlesize = 0;
unsigned int RN_bundlesize = 0;
//  Pipeline Registers
instruction* DE;
std::vector<instruction> DE_v;
instruction* RN;
std::vector<instruction> RN_v;
instruction* RR;
std::vector<instruction> RR_v;
instruction* DI;
std::vector<instruction> DI_v;
instruction* WB;
std::vector<instruction> WB_v;
instruction info_container[10001];
//  Pipeline Register (Memory) Structures
//Issue Queue, build iq_size 1-D array of IQ objects
IssueQueue* IQ;

//Rename Map Table - given that ISA has r0-r66 architectural registers
std::vector<RMTEntry> RMT_vector(67);
//Note: Not modelling ARF - redundant in this case, logic "merged"
//Initialized to both headptr and tailptr pointing to same address
//Chose 500 because I hope it's sufficiently large
ROBEntry* ROB;
const int initial_ROB = 500;
unsigned int ROB_head = 500;
unsigned int ROB_tail = 500;

//Execution List
instruction* execute_list;
std::list <ARF> arf; //not needed? no values?
///////////////////////////////////////////////////////////////////////
/////////////////////////    CONTROL FLAGS & FUNCTIONS    //////////////////////////
////////////////////////    System state control   /////////////////////
//Check to see if all inputs have been read, to control when to stop Fetch
bool isEOF = false;
//If operations underway, cannot finish
bool pipelineBusy = false;

//Stage Logic Control Flags
bool DI_empty();
bool RR_empty();
bool DE_empty();
bool RN_empty();
//moving all instruction container information
void move_to_DI(instruction* RR_s, instruction* DI_d);
void move_to_RR(instruction* RN_s, instruction* RR_d);
void move_to_RN(instruction* DE_d, instruction* RN_d);
std::string curr_stage = " ";
void calculate_timing(std::string stage);
////////////////////////    Stage logic flow control   ////////////////////////
//  IQ
bool IQ_has_ready_instructions();
unsigned int empty_spots_in_IQ();
unsigned long int age_oldest_instruction();
unsigned long int oldest_instr(unsigned long int m_age);
unsigned int find_all_empty_IQ();
//  ROB
unsigned int find_all_empty_ROB();
unsigned int empty_spots_in_ROB();
void next_head(unsigned int current_head);
void next_tail(unsigned int current_tail);
bool ROB_ready_flag();
void create_space_in_ROB();
void allocate_ROB(unsigned int rob_index, unsigned int rr_index);
bool rob_overflow();
bool rob_overflow_tail();
bool rob_overflow_flag = false;
//  RR
bool RR_allready(instruction* src, ROBEntry* dst);
bool RR_ready_flag = false;
bool retire_curr_ROB(unsigned int retire_index);
//  EX
bool EX_has_space();
void wake_up_dependents(std::string location);
int issue_to_here();
//  WB
bool no_WB();
bool WB_has_space();
unsigned int WB_empty_index();
int printCounter = 0;
//      Operational Counters
unsigned int fetched_position = 0;
unsigned int num_EX_ins = 0;
unsigned int num_WB_left = 0;
unsigned int program_cycles = 0;
void print_all_contents();
void print_final_stats();
/////////////////////////////////////////////////////////////////////////////


int main(int argc, char* argv[])
{

    params.rob_size = strtoul(argv[1], NULL, 10);
    params.iq_size = strtoul(argv[2], NULL, 10);
    params.width = strtoul(argv[3], NULL, 10);
    trace_file = argv[4];
    /*    printf("rob_size:%lu "
                "iq_size:%lu "
                "width:%lu "
                "tracefile:%s\n", params.rob_size, params.iq_size, params.width, trace_file);*/
                // Open trace_file in read mode
    unsigned long int exec_stage_size = params.width * 5;
    DE = new instruction[params.width];
    DE_v.resize(params.width);
    RN = new instruction[params.width];
    RN_v.resize(params.width);
    RR = new instruction[params.width];
    RR_v.resize(params.width);
    DI = new instruction[params.width];
    DI_v.resize(params.width);
    IQ = new IssueQueue[params.iq_size];
    execute_list = new instruction[exec_stage_size * 5];
    WB = new instruction[params.width * 5];
    WB_v.resize(params.width);
    ROB = new ROBEntry[params.rob_size];
    arf.resize(100001);
    FP = fopen(trace_file, "r");
    if (FP == NULL)
    {
        // Throw error and exit if fopen() failed
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }


    while (Advance_Cycle()) {
        Retire();
        Writeback();
        Execute();
        Issue();
        Dispatch();
        RegRead();
        Rename();
        Decode();
        if (DE[0].pc == 0 && !isEOF) { //If there are more instructions in the trace file and if DE is empty (can accept a new decode bundle)
            for (unsigned int i = 0; i < params.width; i++) { //fetch up to WIDTH instructions from the trace file
                if (fscanf(FP, "%lx %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF) {
                    Fetch(i);
                }
                else {
                    isEOF = true;
                }
            }
            //Do nothing if either (1) there are no more instructions in the trace file or
            //(2) DE is not empty (cannot accept a new decode bundle).
        }
        program_cycles++;
        //NB: remember to remove the extra cycle
    }
    print_all_contents();
    print_final_stats();
    return 0;
}


// FUNCTIONS
bool Advance_Cycle() {
    // if EOF and no operations underway, end
    if (isEOF && !pipelineBusy) {
        return false;
    }
    else { // else, reset pipelineBusy flag and advance to next cycle
        pipelineBusy = false;
        return true;
    }
}


void Fetch(int i) {
    instruction ins_c;
    // set pipelineBusy to true. pipeline is busy if fetching new instructions
    pipelineBusy = true;

    //fetch instructions into DE
    DE[i].pc = pc;
    DE[i].op = op_type;
    DE[i].dst = dest;
    DE[i].src1 = src1;
    DE[i].src2 = src2;

    DE_v[i].pc = pc;
    DE_v[i].op = op_type;
    DE_v[i].dst = dest;
    DE_v[i].src1 = src1;
    DE_v[i].src2 = src2;
    //as width or less instructions are fetched, keep track of them for use in later stages
    DE_bundlesize = i + 1;
    //store instruction age and increment as more are fetched from the tracefile
    fetched_position++;
    DE[i].age = fetched_position;
    DE_v[i].age = fetched_position;

    //set execution latency based on op code
    if (op_type == 0) {
        DE[i].latency = 1;
    }
    else if (op_type == 1) {
        DE[i].latency = 2;
    }
    else if (op_type == 2) {
        DE[i].latency = 5;
    }

    if (op_type == 0) {
        DE_v[i].latency = 1;
    }
    else if (op_type == 1) {
        DE_v[i].latency = 2;
    }
    else if (op_type == 2) {
        DE_v[i].latency = 5;
    }

    //allocate copy to preserve for output
    info_container[fetched_position].age_p = fetched_position;
    info_container[fetched_position].op_p = op_type;
    info_container[fetched_position].src1_p = src1;
    info_container[fetched_position].src2_p = src2;
    info_container[fetched_position].dst_p = dest;
    info_container[fetched_position].FE_start = program_cycles;
}


void Retire() {
    curr_stage = "Retire";

    //set start cycle and calculate time spent by the instruction in WB
    calculate_timing(curr_stage);
    //check for ready instructions
    if (!ROB_ready_flag()) {
        //none are ready
        //do nothing
        return;
    }
    else {

        //Retire up to WIDTH consecutive “ready” instructions
        for (unsigned int i = 0; i < params.width; i++) {
            //“ready” instructions from the head of the ROB
            if (!ROB_ready_flag()) {
                //do nothing if ins isn't ready (at the head)
                return;
            }
            else {
                //note that retire starts next cycle
                info_container[ROB[ROB_head - initial_ROB].age].RT_cycles = program_cycles - info_container[ROB[ROB_head - initial_ROB].age].RT_start + 1;
                //check if ROB entry at location is valid
                if (ROB[ROB_head - initial_ROB].dst != -1) {

                    if (RMT_vector[ROB[ROB_head - initial_ROB].dst].rob_tag == ROB_head) {
                        //if tag in RMT, clear that entry
                        if (RMT_vector[ROB[ROB_head - initial_ROB].dst].rob) {
                            RMT_vector[ROB[ROB_head - initial_ROB].dst].rob = 0;
                            RMT_vector[ROB[ROB_head - initial_ROB].dst].rob_tag = 0;
                        }
                    }
                }

                //see if there are any instructions fully ready in curr cycle
                //checks if both
                for (unsigned int x = 0; x < params.width; x++) {
                    if (retire_curr_ROB(x) == true) {
                        RR[x].src1 == ROB_head;
                        RR[x].src1_rdy = true;
                        RR[x].src2 == ROB_head;
                        RR[x].src2_rdy = true;
                    }

                    if (RR[x].src1 == ROB_head) {
                        RR[x].src1_rdy = true;
                    }
                    if (RR[x].src2 == ROB_head) {
                        RR[x].src2_rdy = true;
                    }
                }
                //clear an entry
                create_space_in_ROB();
                //increment head ptr to go to next entry
                //circular behavior implemented via overflow flag to set headptr = tailptr
                next_head(ROB_head);
            }
            //since there are ops to do, set pipeline busy high
            pipelineBusy = true;
            //set above to false ONLY when last instruction has been retired, handled in main routine

        }
    }
    return;
}


void Writeback() {
    //store stageName
    curr_stage = "Writeback";
    //set start cycle and calculate time spent by the instruction in EX
    calculate_timing(curr_stage);
    //check if there are any writebacks to be processed
    if (no_WB()) {
        //if there are none
        //do nothing
        return;
    }
    //there are writebacks to process
    else if (!no_WB()) {

        //Process the writeback bundle in WB
        for (unsigned int i = 0; i < params.width * 5; i++) {
            if (no_WB()) {
                //do nothing
                return;
            }
            else {
                //For each instruction in WB, mark the instruction as “ready” in its entry in the ROB.
                ROB[WB[i].dst - initial_ROB].rdy = true;
                //clear the slot in WB register
                WB[i].pc = 0;
                //One WB has been processed completely
                //Decrement counter
                num_WB_left--;
            }
        }
        // set pipelineBusy to true
        pipelineBusy = true;
    }
    return;
}


void Execute() {
    curr_stage = "Execute";
    //update start and duration
    calculate_timing(curr_stage);

    //Check if EX stage has entries
    if (num_EX_ins > 0) {

        // From the execute_list, check for instructions that are finishing execution this cycle
        for (unsigned int i = 0; i < params.width * 5; i++) {

            if (execute_list[i].valid_ex) {
                //decrement execution cycles
                //different based on op_type
                execute_list[i].latency--;
                //execution finishes this cycle if execution latency is used up
                if (execute_list[i].latency == 0) {
                    //there must be space in WB 
                    if (!WB_has_space()) {
                        //if no space, do nothing
                        return;
                    }
                    else {
                        // 1) Remove the instruction from the execute_list.
                        // 2) Add the instruction to WB.
                        // 3) Wakeup dependent instructions (set their source operand ready flags) in the IQ, DI (the dispatch bundle), and
                        // RR (the register-read bundle).
                        //find an empty spot in WB to allocate to 
                        unsigned int allocate_WB_here = WB_empty_index();
                        //allocate at empty index
                        WB[allocate_WB_here] = execute_list[i];
                        num_WB_left++;
                        //wake up dependants
                        wake_up_dependents("IQ");
                        wake_up_dependents("DI");
                        wake_up_dependents("RR");

                        // remove it from execute_list
                        execute_list[i].valid_ex = 0;

                        // decrement num_num_EX_ins
                        num_EX_ins--;
                    }
                }
            }
        }
        // set pipelineBusy to true
        pipelineBusy = true;
    }
}


void Issue() {
    curr_stage = "Issue";
    //set start and duration times for dispatch and issue 
    calculate_timing(curr_stage);

    // if there are ready instructions in the IQ
    if (IQ_has_ready_instructions()) {
        // set pipelineBusy to true

        //Issue up to WIDTH oldest instructions from the IQ.
        for (unsigned int i = 0; i < params.width; i++) {
            pipelineBusy = true;
            int emptyIndex;
            //check IQ for any ready instructions
            if (IQ_has_ready_instructions()) {
                //check if EX has space
                if (!EX_has_space()) {
                    //if no space, do nothing
                    return;
                }
                else {
                    //find an empty slot in EX
                    emptyIndex = issue_to_here();
                }
                //since this will go into EX, increment counter
                num_EX_ins++;
                //make multiple passes through the IQ, each time finding the next oldest ready instruction and then issuing it
                instruction move_to_Ex;
                //get age of oldest instruction
                unsigned long int ins_age = age_oldest_instruction();
                //get location of oldest instruction
                unsigned long int ins_index = oldest_instr(ins_age);
                // To issue an instruction: 
                //1) Remove the instruction from the IQ.
                move_to_Ex = IQ[ins_index].instr;
                move_to_Ex.age = IQ[ins_index].instr.age;
                //now invalidate the IQ entry
                IQ[ins_index].valid = 0;
                // 2) Add the instruction to the execute_list. Set a timer for the instruction in the execute_list that
                // will allow you to model its execution latency.
                execute_list[emptyIndex] = move_to_Ex;
                //set as valid entry in EX
                execute_list[emptyIndex].valid_ex = 1;
            }
            //IQ has no ready instructions, move on
            else return;
        }
        pipelineBusy = true;
    }
    return;
}


void Dispatch() {
    curr_stage = "Dispatch";
    calculate_timing(curr_stage);

    //check If DI contains a dispatch bundle:
    if (DI_empty()) {
        //if no bundle, do nothing
        return;
    }
    else if (!(DI_empty())) {

        //check for free IQ entries
        unsigned int IQ_space = empty_spots_in_IQ();

        //If no space
        if (IQ_space == 0) {
            //do nothing, return
            return;
        }
        else {
            //find number of free IQ entries
            unsigned int available_spots = find_all_empty_IQ();
            //If the number of free IQ entries is greater than or equal to the size of the dispatch bundle in DI
            if ((available_spots >= DI_bundlesize) && IQ_space != 0) {
                //dispatch all instructions from DI to the IQ.
                unsigned int i = 0;
                unsigned int j = 0;
                while (i < DI_bundlesize) {
                    if (IQ[j].valid == 0) {
                        // add to IQ
                        IQ[j].valid = 1;
                        IQ[j].instr = DI[i];
                        // remove from DI
                        DI[i].pc = 0;
                        i++;
                    }
                    j++;
                }
            }
        }
        pipelineBusy = true;
    }
    return;
}


void RegRead() {
    curr_stage = "RegRead";
    calculate_timing(curr_stage);
    //check If RR contains a register-read bundle:
    if ((RR_empty())) {
        //if it doesn't do nothing
        return;
    }
    else if (!RR_empty()) {
        //check if DI is not empty
        if (!DI_empty()) {
            //cannot accept new dispatch bundle, do nothing
            return;
        }
        //if DI is empty (can accept a new dispatch bundle)
        else if (DI_empty()) {
            //process the RegRead bundle
            // for every instruction in bundle

            for (unsigned int i = 0; i < RR_bundlesize; i++) {

                // if src1 is a ROB tag AND IT'S STILL VALID ** AND src1_rdy NOT ALREADY SET
                if (RR[i].src1 >= initial_ROB) {
                    //Check if valid entry exists by checking ROB if pc exists in that slot
                    //if pc exists in ROB, then it's not ins value, its a tag
                    if (ROB[RR[i].src1 - initial_ROB].pc != 0 && ROB[RR[i].src1 - initial_ROB].rdy) {
                        RR[i].src1_rdy = true;
                    }
                }
                else RR[i].src1_rdy = true;//implied ready

                if (RR[i].src2 >= initial_ROB) {
                    //Check if valid entry exists by checking ROB if pc exists in that slot
                    //if pc exists in ROB, then it's not ins value, its a tag
                    if (ROB[RR[i].src2 - initial_ROB].pc != 0 && ROB[RR[i].src2 - initial_ROB].rdy) {
                        RR[i].src2_rdy = true;
                    }
                }
                else RR[i].src2_rdy = 1;//implied ready
            }
            //sources are ready
            move_to_DI(RR, DI);
            pipelineBusy = true;
        }
    }
    return;
}


void Rename() {
    curr_stage = "Rename";
    calculate_timing(curr_stage);
    //check if RN contains a rename bundle
    if (RN_empty()) {
        //if it does not, do nothing
        return;
    }
    else {
        //check if RR is empty
        if (!RR_empty()) {
            //RR is not empty (cannot accept a new register-read bundle)
            //do nothing
            return;
        }
        else {
            //check if ROB has any empty spots at all
            unsigned int ROB_space = empty_spots_in_ROB();

            //if ROB has no space
            if (ROB_space == 0) {
                //do nothing, return
                return;
            }
            else {
                //find all empty spots in ROB
                unsigned int total_space_in_ROB = find_all_empty_ROB();
                //check if ROB does not have enough free entries to
                // accept the entire rename bundle,
                if (total_space_in_ROB >= RN_bundlesize && ROB_space != 0) {
                    //move to RR
                    move_to_RR(RN, RR);

                    for (unsigned int i = 0; i < RR_bundlesize; i++) {
                        //(1) allocate an entry in the ROB for the instruction, 
                        //(2) rename its source registers
                        //(3) rename its destination register (if it has one).
                        unsigned int ROBindex = ROB_tail - initial_ROB;
                        allocate_ROB(ROBindex, i);
                        ROB[ROBindex].rdy = false;

                        //check and allocate for each source from the RMT
                        if (RR[i].src1 == -1) {
                            RR[i].src1 = RR[i].src1;
                        }
                        else {
                            if (RMT_vector[RR[i].src1].rob) {
                                RR[i].src1 = RMT_vector[RR[i].src1].rob_tag;
                            }
                            else {
                                RR[i].src1 = RR[i].src1;
                            }
                        }

                        if (RR[i].src2 == -1) {
                            RR[i].src2 = RR[i].src2;
                        }
                        else {
                            if (RMT_vector[RR[i].src2].rob) {
                                RR[i].src2 = RMT_vector[RR[i].src2].rob_tag;
                            }
                            else {
                                RR[i].src2 = RR[i].src2;
                            }
                        }

                        if (RR[i].dst != -1) {
                            RMT_vector[RR[i].dst].rob_tag = ROB_tail;
                            RMT_vector[RR[i].dst].rob = 1;
                        }

                        //add valid ROB tag
                        RR[i].dst = ROB_tail;

                        //decide whether to increment or overflow ROB buffer
                        next_tail(ROB_tail);
                    }
                    pipelineBusy = true;
                }
            }
        }
    }
    return;
}


void Decode() {
    curr_stage = "Decode";
    calculate_timing(curr_stage);
    //check If DE contains a decode bundle
    if (DE_empty()) {
        //if it does not
        //do nothing
        return;
    }
    else {
        //check if RN is not empty (cannot accept a new rename bundle)
        if (!RN_empty()) {
            //do nothing
            return;
        }
        //If RN is empty (can accept a new rename bundle),
        else if (RN_empty()) {
            //advance the decode bundle from DE to RN.
            move_to_RN(DE, RN);
            pipelineBusy = true;
        }
    }
    return;
}


//used in RegRead
void move_to_DI(instruction* RR_s, instruction* DI_d) {
    for (unsigned int i = 0; i < RR_bundlesize; i++) {
        DI_d[i].pc = RR_s[i].pc;
        DI_d[i].op = RR_s[i].op;
        DI_d[i].dst = RR_s[i].dst;
        DI_d[i].src1 = RR_s[i].src1;
        DI_d[i].src2 = RR_s[i].src2;
        DI_d[i].src1_rdy = RR_s[i].src1_rdy;
        DI_d[i].src2_rdy = RR_s[i].src2_rdy;
        DI_d[i].latency = RR_s[i].latency;
        DI_d[i].age = RR_s[i].age;
        RR_s[i].pc = 0;
    }
    DI_bundlesize = RR_bundlesize;
}

//used in Rename
void move_to_RR(instruction* RN_s, instruction* RR_d) {
    for (unsigned int i = 0; i < RN_bundlesize; i++) {
        RR_d[i].pc = RN_s[i].pc;
        RR_d[i].op = RN_s[i].op;
        RR_d[i].dst = RN_s[i].dst;
        RR_d[i].src1 = RN_s[i].src1;
        RR_d[i].src2 = RN_s[i].src2;
        RR_d[i].src1_rdy = RN_s[i].src1_rdy;
        RR_d[i].src2_rdy = RN_s[i].src2_rdy;
        RR_d[i].latency = RN_s[i].latency;
        RR_d[i].age = RN_s[i].age;
        RN_s[i].pc = 0;
    }
    RR_bundlesize = RN_bundlesize;
}

//used in Decode
void move_to_RN(instruction* DE_d, instruction* RN_d) {
    for (unsigned int i = 0; i < DE_bundlesize; i++) {
        RN_d[i].pc = DE_d[i].pc;
        RN_d[i].op = DE_d[i].op;
        RN_d[i].dst = DE_d[i].dst;
        RN_d[i].src1 = DE_d[i].src1;
        RN_d[i].src2 = DE_d[i].src2;
        RN_d[i].src1_rdy = DE_d[i].src1_rdy;
        RN_d[i].src2_rdy = DE_d[i].src2_rdy;
        RN_d[i].latency = DE_d[i].latency;
        RN_d[i].age = DE_d[i].age;
        DE_d[i].pc = 0;
    }
    RN_bundlesize = DE_bundlesize;
}


unsigned int find_all_empty_ROB() {
    unsigned int num_empty_spots = 0;
    for (unsigned int i = 0; i < params.rob_size; i++) {
        if (ROB[i].pc == 0) {
            num_empty_spots++;
        }
    }
    return num_empty_spots;
}

unsigned int empty_spots_in_ROB() {
    unsigned int empty_spots_in_ROB = 0;
    for (unsigned int i = 0; i < params.rob_size; i++) {
        //check for invalid entries, 
        if (ROB[i].pc == 0) {
            empty_spots_in_ROB++;
        }
    }
    return empty_spots_in_ROB;
}


unsigned int find_all_empty_IQ() {
    unsigned int num_empty_spots = 0;
    for (unsigned int i = 0; i < params.iq_size; i++) {
        if (IQ[i].valid == 0) {
            num_empty_spots++;
        }
    }
    return num_empty_spots;
}


unsigned int empty_spots_in_IQ() {
    unsigned int empty_spots_in_IQ = 0;
    for (unsigned int i = 0; i < params.iq_size; i++) {
        //check for invalid entries, 
        if (IQ[i].valid == 0) {
            empty_spots_in_IQ++;
        }
    }
    return empty_spots_in_IQ;
}

bool IQ_has_ready_instructions() {
    for (unsigned long int i = 0; i < params.iq_size; i++) {
        //check if entry is valid first
        if (IQ[i].valid) {
            //check that BOTH ready bits are ready
            if (IQ[i].instr.src1_rdy == true && IQ[i].instr.src2_rdy == true) {
                return true;
                break;
            }
        }
    }
    return false;
}

bool EX_has_space() {
    for (unsigned long int i = 0; i < (params.width * 5); i++) {
        if (execute_list[i].valid_ex == 0) {
            return true;
            break;
        }
    }
    return false;
}


int issue_to_here() {
    // return location of any empty slot in EX
    for (unsigned int i = 0; i < (params.width * 5); i++) {
        if (execute_list[i].valid_ex == 0) {
            return i;
        }
    }
}

unsigned long int age_oldest_instruction() {
    //note: age is incremented as NEW instructions are added
    //so we need the minimum age
    unsigned long int curr_age = 0;
    //unsigned long long int index = 0;
    for (unsigned long int i = 0; i < params.iq_size; i++) {
        if (IQ[i].valid) {
            if ((IQ[i].instr.src1_rdy == true) && (IQ[i].instr.src2_rdy == true)) {
                //set for first 
                if (curr_age == 0) {
                    curr_age = IQ[i].instr.age;
                }
                if (IQ[i].instr.age < curr_age || IQ[i].instr.age == curr_age) {
                    curr_age = IQ[i].instr.age;
                }
            }
        }
    }
    return curr_age;
}

unsigned long int oldest_instr(unsigned long int m_age) {
    unsigned long int return_index = 0;
    for (unsigned int i = 0; i < params.iq_size; i++) {
        // if instr is valid and both source regs are ready
        if (IQ[i].valid && IQ[i].instr.src1_rdy && IQ[i].instr.src2_rdy) {
            if (IQ[i].instr.age == m_age) {
                return_index = i;
            }
        }
    }
    return return_index;
}


bool WB_has_space() {
    for (unsigned int i = 0; i < params.width * 5; i++) {
        if (WB[i].pc == 0) {
            return true;
            break;
        }
    }
    return false;
}

unsigned int WB_empty_index() {
    //since I already checked if there is space, not adding other control
    for (unsigned int i = 0; i < params.width * 5; i++) {
        if (WB[i].pc == 0) {
            return i;
        }
    }
}


void next_head(unsigned int current_head) {
    ROB_head++;
    //if overflow
    if (rob_overflow()) {
        //circular behavior
        ROB_head = initial_ROB;
    }
}

void next_tail(unsigned int current_tail) {
    ROB_tail++;
    //if overflow
    if (rob_overflow_tail()) {
        //circular behavior
        ROB_tail = initial_ROB;
    }
}

void calculate_timing(std::string curr_stage) {
    //The ending point (current cycle) for a stage is the starting point for the next stage
    //Subtracting provides the duration

    if (curr_stage == "Retire") {
        for (unsigned int u = 0; u < params.rob_size; u++) {
            if (info_container[ROB[u].age].RT_start == 0 && info_container[ROB[u].age].WB_cycles == 0) {
                if (ROB[u].rdy) {
                    //set start time for current stage
                    info_container[ROB[u].age].RT_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[ROB[u].age].WB_cycles = program_cycles - info_container[ROB[u].age].WB_start;
                }
            }
        }
    }
    else if (curr_stage == "Decode") {
        for (unsigned int i = 0; i < DE_bundlesize; i++) {
            if (DE[0].pc != 0) {
                if (info_container[DE[i].age].DE_start == 0 && info_container[DE[i].age].FE_cycles == 0) {
                    //set start time for current stage
                    info_container[DE[i].age].DE_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[DE[i].age].FE_cycles = program_cycles - info_container[DE[i].age].FE_start;
                }
            }
        }
    }
    else if (curr_stage == "RegRead") {
        for (unsigned int e = 0; e < RR_bundlesize; e++) {
            if (info_container[RR[e].age].RR_start == 0 && info_container[RR[e].age].RN_cycles == 0) {
                if (RR[0].pc != 0) {
                    //set start time for current stage
                    info_container[RR[e].age].RR_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[RR[e].age].RN_cycles = program_cycles - info_container[RR[e].age].RN_start;
                }
            }
        }
    }
    else if (curr_stage == "Rename") {
        for (unsigned int u = 0; u < RN_bundlesize; u++) {
            if (RN[0].pc != 0) {
                if (info_container[RN[u].age].RN_start == 0) {
                    if (info_container[RN[u].age].DE_cycles == 0) {
                        //set start time for current stage
                        info_container[RN[u].age].RN_start = program_cycles;
                        //calculate time spent in previous stage
                        info_container[RN[u].age].DE_cycles = program_cycles - info_container[RN[u].age].DE_start;
                    }
                }
            }
        }
    }
    else if (curr_stage == "Writeback") {
        for (unsigned int u = 0; u < params.width * 5; u++) {
            if (num_WB_left) {
                if (info_container[WB[u].age].WB_start == 0 && info_container[WB[u].age].EX_cycles == 0) {
                    //set start time for current stage
                    info_container[WB[u].age].WB_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[WB[u].age].EX_cycles = program_cycles - info_container[WB[u].age].EX_start;
                }
            }
        }
    }
    else if (curr_stage == "Execute") {
        for (unsigned int u = 0; u < params.width * 5; u++) {
            if (info_container[execute_list[u].age].EX_start == 0 && info_container[execute_list[u].age].IS_cycles == 0) {
                if (num_EX_ins) {
                    //set start time for current stage
                    info_container[execute_list[u].age].EX_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[execute_list[u].age].IS_cycles = program_cycles - info_container[execute_list[u].age].IS_start;
                }
            }
        }
    }
    else if (curr_stage == "Dispatch") {
        for (unsigned int e = 0; e < DI_bundlesize; e++) {
            if (DI[0].pc != 0) {
                if (info_container[DI[e].age].DI_start == 0 && info_container[DI[e].age].RR_cycles == 0) {
                    //set start time for current stage
                    info_container[DI[e].age].DI_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[DI[e].age].RR_cycles = program_cycles - info_container[DI[e].age].RR_start;
                }
            }
        }
    }
    else if (curr_stage == "Issue") {
        for (unsigned int k = 0; k < params.iq_size; k++) {
            if (info_container[IQ[k].instr.age].IS_start == 0 && info_container[IQ[k].instr.age].DI_cycles == 0) {
                if (IQ[k].valid) {
                    //set start time for current stage
                    info_container[IQ[k].instr.age].IS_start = program_cycles;
                    //calculate time spent in previous stage
                    info_container[IQ[k].instr.age].DI_cycles = program_cycles - info_container[IQ[k].instr.age].DI_start;
                }
            }
        }
    }
}


bool ROB_ready_flag() {
    if (ROB[ROB_head - initial_ROB].rdy) {
        return true;
    }
    else {
        return false;
    }
}

bool retire_curr_ROB(unsigned int retire_index) {
    if (RR[retire_index].src1 == ROB_head && RR[retire_index].src2 == ROB_head) {
        return true;
    }
    return false;
}

void create_space_in_ROB() {
    ROB[ROB_head - initial_ROB].pc = 0;
    ROB[ROB_head - initial_ROB].rdy = false;
    ROB[ROB_head - initial_ROB].dst = 0;
}

void allocate_ROB(unsigned int rob_index, unsigned int rr_index) {
    ROB[rob_index].pc = RR[rr_index].pc;
    ROB[rob_index].age = RR[rr_index].age;
    ROB[rob_index].dst = RR[rr_index].dst;
}

bool rob_overflow() {
    if (ROB_head >= (initial_ROB + params.rob_size)) {
        return true;
        rob_overflow_flag = true;
    }
    else {
        return false;
        rob_overflow_flag = false;
    }
}

bool rob_overflow_tail() {
    if (ROB_tail >= (initial_ROB + params.rob_size)) {
        return true;
    }
    else {
        return false;
    }
}

bool DI_empty() {
    if (DI[0].pc == 0) {
        return true;
    }
    else {
        return false;
    }
}

bool RR_empty() {
    if (RR[0].pc == 0) {
        return true;
    }
    else {
        return false;
    }
}

bool DE_empty() {
    if (DE[0].pc != 0) {
        return false;
    }
    else {
        return true;
    }
}

bool RN_empty() {
    if (RN[0].pc != 0) {
        return false;
    }
    else {
        return true;
    }
}

bool no_WB() {
    if (num_WB_left > 0) {
        return false;
    }
    else {
        return true;
    }
}

void wake_up_dependents(std::string location) {

    if (location == "IQ") {
        for (unsigned int i = 0; i < params.width * 5; i++) {
            if (execute_list[i].valid_ex) {
                //if its done, wake up dependants 
                if (execute_list[i].latency == 0) {
                    for (unsigned int z = 0; z < params.iq_size; z++) {
                        if (IQ[z].instr.src1 == execute_list[i].dst)
                            IQ[z].instr.src1_rdy = true;
                        if (IQ[z].instr.src2 == execute_list[i].dst)
                            IQ[z].instr.src2_rdy = true;
                    }
                }
            }
        }
    }
    else if (location == "DI") {
        for (unsigned int i = 0; i < params.width * 5; i++) {
            if (execute_list[i].valid_ex) {
                //if its done, wake up dependants
                if (execute_list[i].latency == 0) {
                    for (unsigned int y = 0; y < params.width; y++) {
                        if (DI[y].src1 == execute_list[i].dst)
                            DI[y].src1_rdy = true;
                        if (DI[y].src2 == execute_list[i].dst)
                            DI[y].src2_rdy = true;
                    }
                }
            }
        }
    }
    else if (location == "RR") {
        for (unsigned int i = 0; i < params.width * 5; i++) {
            if (execute_list[i].valid_ex) {
                //if its done, wake up dependants
                if (execute_list[i].latency == 0) {
                    for (unsigned int x = 0; x < params.width; x++) {
                        if (RR[x].src1 == execute_list[i].dst)
                            RR[x].src1_rdy = true;
                        if (RR[x].src2 == execute_list[i].dst)
                            RR[x].src2_rdy = true;
                    }
                }
            }
        }
    }
}


void print_all_contents() {
    for (unsigned int i = 1; i <= fetched_position; i++) {
        std::cout << i - 1 << " fu{" << info_container[i].op_p << "} src{" << info_container[i].src1_p << "," << info_container[i].src2_p << "} " << "dst{" << info_container[i].dst_p;
        std::cout << "} FE{" << info_container[i].FE_start << "," << info_container[i].FE_cycles << "} DE{";
        std::cout << info_container[i].DE_start << "," << info_container[i].DE_cycles << "} RN{" << info_container[i].RN_start << "," << info_container[i].RN_cycles << "} RR{";
        std::cout << info_container[i].RR_start << "," << info_container[i].RR_cycles << "} DI{" << info_container[i].DI_start << "," << info_container[i].DI_cycles << "} IS{";
        std::cout << info_container[i].IS_start << "," << info_container[i].IS_cycles << "} EX{" << info_container[i].EX_start << "," << info_container[i].EX_cycles << "} WB{";
        std::cout << info_container[i].WB_start << "," << info_container[i].WB_cycles << "} RT{" << info_container[i].RT_start << "," << info_container[i].RT_cycles << "} " << std::endl;
    }
}



void print_final_stats() {
    std::cout << "# === Simulator Command =========" << std::endl;
    std::cout << "# ./sim " << params.rob_size << " " << params.iq_size << " " << params.width << " " << trace_file << std::endl;
    std::cout << "# === Processor Configuration ===" << std::endl;
    std::cout << "# ROB_SIZE = " << params.rob_size << std::endl;
    std::cout << "# IQ_SIZE  = " << params.iq_size << std::endl;
    std::cout << "# WIDTH    = " << params.width << std::endl;
    std::cout << "# === Simulation Results ========" << std::endl;
    std::cout << "# Dynamic Instruction Count    = " << fetched_position << std::endl;
    std::cout << "# Cycles                       = " << program_cycles - 1 << std::endl;
    printf("# Instructions Per Cycle (IPC) = %.2f\n", (float)(fetched_position) / (program_cycles - 1));
    //float IPC = fetched_position / (program_cycles - 1);
    //std::cout << "# Instructions Per Cycle (IPC) = " << std::setprecision(3) << (float)IPC << std::endl;
}