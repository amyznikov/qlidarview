/*
 * c_pcap_file.h
 *
 *  Created on: Mar 11, 2022
 *      Author: amyznikov
 */

#ifndef __c_pcap_file_h__
#define __c_pcap_file_h__

#include <string>
#include <pcap/pcap.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>


#pragma pack(push,1)

/* For pcap link types see
 *  <https://www.ietf.org/id/draft-ietf-opsawg-pcap-00.html>
 */

/**
 * c_en10mb_header
 * DLT_EN10MB
 *
 *  10Mb/s ethernet header + ip + udp
 *  48 bytes
 * */
struct c_en10mb_header
{
  struct ether_header eth;
  struct ip ip;
  struct udphdr udp;
};


/**
 * c_bsd_loopback_header
 * DLT_NULL
 *
 * BSD loopback encapsulation header
 **/
struct c_bsd_loopback_header {
  uint32_t protocol; // AF_INET etc
  struct ip ip;
  struct udphdr udp;
};

/**
 * c_pcap_data_header
 * */
union c_pcap_data_header
{
  struct c_en10mb_header en10mb;
  struct c_bsd_loopback_header loopbak;
};


#pragma pack(pop)


/**
 * c_pcap_reader
 * Class-wraper around of pcap reading functions
 * */
class c_pcap_reader {
public:

  /* Default c'tor */
  c_pcap_reader();

  /* Default d'tor, close file on exit */
  ~c_pcap_reader();

  /** Get current pcap file name */
  const std::string & filename() const;

  /** Get current pcap options */
  const std::string & options() const;

  /** Get last error message from a pcap function */
  const std::string & errmsg () const;

  /* Access to pcap file handle */
  pcap_t * pcap() const;

  /* Get time stamp precison, one of
   * PCAP_TSTAMP_PRECISION_MICRO or PCAP_TSTAMP_PRECISION_NANO
   * */
  uint precision() const;

  int data_header_size() const;

  int datalinktype() const;

  /* Open pcap file for read */
  bool open(const std::string & filename, const std::string & filter = "",
      uint precision = PCAP_TSTAMP_PRECISION_MICRO);

  /* Check if file is open */
  bool is_open() const;

  void close();

  /** read()
   * Read the next packet from pcap file.
   * The struct pcap_pkthdr and the packet data are not to be freed by the caller,
   * and are not guaranteed to be valid after the next call */
  int read(const pcap_pkthdr **pkt_header,
      const uint8_t **pkt_data);

  /** read()
   * Read the next packet from pcap file.
   * The struct pcap_pkthdr and the packet data are not to be freed by the caller,
   * and are not guaranteed to be valid after the next call */
  int read(const pcap_pkthdr ** pkt_header,
      const c_pcap_data_header ** data_header,
      const uint8_t ** payload);

  /** Set current read file position in bytes*/
  long seek(long position, int whence = SEEK_SET);

  /** Get current read file position in bytes*/
  long tell() const;

protected:
  pcap_t * pcap_ = nullptr;
  std::string filename_;
  std::string options_;
  std::string errmsg_;
  uint precision_ = PCAP_TSTAMP_PRECISION_MICRO;
  //uint frame_header_length_ = 0;
  int datalinktype_ = 0;
  int data_header_size_ = -1;
};

/**
 * Error messages from pcap API
 * */
const char* pcap_errmsg(int status);

#endif /* __c_pcap_file_h__ */
