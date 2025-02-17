/*
 * @brief
 *
 */
#ifndef TCPSERV_H_
#define TCPSERV_H_

#define TCP_SERVER_PORT 9001

/*
 * @brief
 */
typedef enum tcp_server_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
}tcp_server;


/* Public function prototype. */
void tcp_server_init(void);

#endif /* TCPSERV_H_ */
