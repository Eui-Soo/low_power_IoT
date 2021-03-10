#ifndef __IPC_H__
#define __IPC_H__


#define MAX_IPC_PKT_SIZE	4096

typedef struct
{
	uint8_t msg_id;
	uint16_t msg_len;
}IPC_PKT_HDR;


typedef struct
{
	IPC_PKT_HDR hdr;
	uint8_t data[MAX_IPC_PKT_SIZE];
}IPC_PKT;


#define IPC_GET_ID(ipc)				((ipc)->hdr.msg_id)		
#define IPC_GET_LEN(ipc)				((ipc)->hdr.msg_len)		
#define IPC_GET_DATA_PTR(ipc)	((ipc)->data)		


#define IPC_SET_ID(ipc, val)				((ipc)->hdr.msg_id = val)		
#define IPC_SET_LEN(ipc, val)				((ipc)->hdr.msg_len = val)		


#endif

