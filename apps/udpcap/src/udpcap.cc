/*
 * udpcap.cc
 *
 *  Created on: Mar 26, 2022
 *      Author: amyznikov
 */
#include <stdio.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <string.h>
#include <string>


// sudo tcpdump -vv -i any -u "src 192.168.2.201 and udp" -w test2.pcap

int main(int argc, char *argv[])
{
  std::string devip_str = "192.168.2.201";
  uint16_t port = 2368;

  sockaddr_in my_addr;                     // my address information
  struct in_addr devip;
  int sockfd = -1;

  inet_aton(devip_str.c_str(),&devip);

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if( sockfd == -1 ) {
    perror("socket");
    return 1;
  }

  memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
  my_addr.sin_family = AF_INET;            // host byte order
  my_addr.sin_port = htons(port);          // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  // compatibility with Spot Core EAP, reuse port 2368
  int val = 1;
  if( setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1 ) {
    perror("socketopt");
    return 1;
  }

  if( bind(sockfd, (sockaddr*) &my_addr, sizeof(sockaddr)) == -1 ) {
    perror("bind");
    return 1;
  }

  if( fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0 ) {
    perror("non-block");
    return 1;
  }

  struct pollfd fds[1];
  fds[0].fd = sockfd;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  const uint max_packet_size = 2048;
  uint8_t pkt[max_packet_size];

  for ( int i = 0; i < 100; ++i ) {
    while (true) {

      // poll() until input available
      do {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if( retval < 0 ) {            // poll() error?
          if( errno != EINTR ) {
            fprintf(stderr, "poll() error: %s\n", strerror(errno));
          }
          return 1;
        }
        if( retval == 0 ) {  // poll() timeout?
          fprintf(stderr, "Velodyne poll() timeout\n");
          return 1;
        }
        if( (fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL) ) { // device error?
          fprintf(stderr, "poll() reports Velodyne error\n");
          return -1;
        }
      } while ((fds[0].revents & POLLIN) == 0);

      // Receive packets that should now be available from the
      // socket using a blocking read.
      ssize_t nbytes =
          recvfrom(sockfd, pkt,
              max_packet_size, 0,
              (sockaddr*) &sender_address,
              &sender_address_len);

      if( nbytes < 0 ) {
        if( errno != EWOULDBLOCK ) {
          perror("recvfail");
          return 1;
        }
      }
      else {
        fprintf(stderr, "packet read: nbytes=%zd\n", nbytes);
      }
    }
  }

  close(sockfd);

  return 0;
}
