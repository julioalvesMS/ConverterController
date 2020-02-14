#ifndef SRC_IPC_PROTOCOL_H_
#define SRC_IPC_PROTOCOL_H_

typedef enum CommunicationCommand
{
    None = 0,

    IncreaseDacGroup,
    DecreaseDacGroup,

} CommunicationCommand;

#endif /* SRC_IPC_PROTOCOL_H_ */
