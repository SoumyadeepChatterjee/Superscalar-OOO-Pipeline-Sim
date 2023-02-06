#ifndef SIM_PROC_H
#define SIM_PROC_H

typedef struct proc_params {
	unsigned long int rob_size;
	unsigned long int iq_size;
	unsigned long int width;
}proc_params;

//data container for everything
typedef struct instruction_container {
	unsigned int age = 0;
	unsigned long int pc = 0;
	int op = 0;
	int dst = 0;
	int src1 = 0;
	int src2 = 0;
	bool src1_rdy = false;
	bool src2_rdy = false;
	unsigned int age_p = 0;
	int op_p = 0;
	int src1_p = 0;
	int src2_p = 0;
	int dst_p = 0;
	unsigned int FE_start = 0;
	unsigned int FE_cycles = 0;
	unsigned int DE_start = 0;
	unsigned int DE_cycles = 0;
	unsigned int RN_start = 0;
	unsigned int RN_cycles = 0;
	unsigned int RR_start = 0;
	unsigned int RR_cycles = 0;
	unsigned int DI_start = 0;
	unsigned int DI_cycles = 0;
	unsigned int IS_start = 0;
	unsigned int IS_cycles = 0;
	unsigned int EX_start = 0;
	unsigned int EX_cycles = 0;
	unsigned int WB_start = 0;
	unsigned int WB_cycles = 0;
	unsigned int RT_start = 0;
	unsigned int RT_cycles = 0;
	unsigned int latency = 0;
	bool valid_ex = 0;
} instruction;

//Rename Map Table
typedef struct RMTEntry {
	unsigned int rob = 0;
	unsigned int rob_tag = 0;
}RMTEntry;

//ROB - headptr and tailptr are used to keep track of the oldest and newest instructions
typedef struct ROBEntry {
	int dst = -1;
	bool rdy = 0;
	unsigned long int pc = 0;
	unsigned int age = 0;
}ROBEntry;

//Issue Queue, has own unique valid bit
typedef struct IssueQueue {
	unsigned int valid = 0;
	instruction instr;
}IssueQueue;

typedef struct ARF {
	unsigned int value = 0;
}ARF;

#endif
