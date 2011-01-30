#include "msg.h"

#define LARGESTRING 1024

#define SPERW     (7 * 24 * 3600)
#define SPERD     (24 * 3600)
#define SPERH     (3600)
#define SPERM     (60)

#define SQN_PRT_MODULE_NAME	"wimax_prt"

void  printTime32(u_char *data);

void sqn_pr_info_dump(char *prefix, unsigned char *data, unsigned int len) {
   	unsigned int i = 0, pos = 0, temp = 0;
	unsigned int width = 16;
	unsigned int len_ = (unsigned int)(len);

    char buf[LARGESTRING];

	int opCode = 0;
	int bHandle = 0;

    // sequans_xxx: RX PDU: 0000 ff ff ff ff ff ff 00 1e 90 21 0b d4 08 00 45 00 
	// while (i < len_) { // Andrew 0903
    if (i < len_) {   
		if (i % width == 0)
        {
			if (len_ >= 40 && !bHandle) { // ARP [						
				if ( (((unsigned char *)(data))[i+12] == 0x08) && 
					 (((unsigned char *)(data))[i+13] == 0x06) ) {
					
					bHandle = 1;
						
					// Opcode
					opCode = 0;		
					if ( (((unsigned char *)(data))[i+20] == 0x00) && 
						 (((unsigned char *)(data))[i+21] == 0x01) ) {
						printk(KERN_INFO "%s: [ARP request] - ", SQN_PRT_MODULE_NAME);  
						opCode = 1;
					}
					else if ( (((unsigned char *)(data))[i+20] == 0x00) && 
						 (((unsigned char *)(data))[i+21] == 0x02) ) {
						printk(KERN_INFO "%s: [ARP reply] - ", SQN_PRT_MODULE_NAME);  
						opCode = 2;
					}
					
					if (opCode == 1) { // request
						printk("Who has ");  
						for (pos=38; pos<42; pos++) {
							if (pos<41) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d Tell ", ((unsigned char *)(data))[pos]);
						}
						for (pos=28; pos<32; pos++) {
							if (pos<31) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
						}
					}
					else if (opCode == 2) { // reply									
						for (pos=28; pos<32; pos++) {
							if (pos<31) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d is at ", ((unsigned char *)(data))[pos]);
						}						

						for (pos=22; pos<28; pos++) {
							if (pos<27) 
								printk("%02x:", ((unsigned char *)(data))[pos]);							
							else
								printk("%02x, ", ((unsigned char *)(data))[pos]);
						}
					}

                    // Destination MAC
					printk("Dst MAC: ");  
					for (pos=0; pos<6; pos++) {
						if (pos<5) 
								printk("%02x:", ((unsigned char *)(data))[pos]);							
							else
								printk("%02x, ", ((unsigned char *)(data))[pos]);
					}					

					// Source MAC
					printk("Src MAC: ");  
					for (pos=6; pos<12; pos++) {
						if (pos<11) 
								printk("%02x:", ((unsigned char *)(data))[pos]);							
							else
								printk("%02x\n", ((unsigned char *)(data))[pos]);
					}
					
				}				
			} // ARP ]

            if (len_ >= 34 && !bHandle) { // ICMP [	
				// IP: 0x0800
				if ( (((unsigned char *)(data))[i+12] == 0x08) && 
					 (((unsigned char *)(data))[i+13] == 0x00) &&
                     (((unsigned char *)(data))[i+23] == 0x01) 
                   ) {			
					bHandle = 1;	
					if ( (((unsigned char *)(data))[i+34] == 0x00) ) {
						printk(KERN_INFO "%s: [ICMP] Echo Reply, ", SQN_PRT_MODULE_NAME);
					}
                    else if ( (((unsigned char *)(data))[i+34] == 0x03) ) {
						printk(KERN_INFO "%s: [ICMP] Destination Unreachable, ", SQN_PRT_MODULE_NAME);
					}
                    else if ( (((unsigned char *)(data))[i+34] == 0x05) ) {
						printk(KERN_INFO "%s: [ICMP] Redirect, ", SQN_PRT_MODULE_NAME);	
					}
                    else if ( (((unsigned char *)(data))[i+34] == 0x08) ) {
						printk(KERN_INFO "%s: [ICMP] Echo Request, ", SQN_PRT_MODULE_NAME);
					}
                    else if ( (((unsigned char *)(data))[i+34] == 0x09) ) {
						printk(KERN_INFO "%s: [ICMP] Router Adventisement, ", SQN_PRT_MODULE_NAME);
					}

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					
                    
                    // Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d\n", ((unsigned char *)(data))[pos]);
					}			
				}				
			} // ICMP ]

			if (len_ >= 300 && !bHandle) { // DHCP [		
				// IP: 0x0800, UDP: 0x11, port: 0x0044, 0x0043
				if ( (((unsigned char *)(data))[i+12] == 0x08) && 
					 (((unsigned char *)(data))[i+13] == 0x00) &&
                     (((unsigned char *)(data))[i+23] == 0x11) &&

					 (
					   (((unsigned char *)(data))[i+34] == 0x00) &&
					   ((((unsigned char *)(data))[i+35] == 0x44) || (((unsigned char *)(data))[i+35] == 0x43)) &&
					   (((unsigned char *)(data))[i+36] == 0x00) &&
					   ((((unsigned char *)(data))[i+37] == 0x43) || (((unsigned char *)(data))[i+37] == 0x44))
					 ) 
                   ) {
				
					bHandle = 1;
					pos = 282;

					while (pos < len_ && data[pos] != 255) { // while option [					

						switch ( ((unsigned char *)(data))[pos] ) { // Option case [
						 
						  case 0:     // pad
								break;					
					
                          case  1:    // Subnetmask                                							        
                                printk(KERN_INFO "%s: Subnet Mask: ", SQN_PRT_MODULE_NAME);  
                                for (temp=0; temp<4; temp++) {
                                    if (temp<3) 
                                            printk("%d.", ((unsigned char *)(data))[pos+2+temp]);							
                                        else
                                            printk("%d\n", ((unsigned char *)(data))[pos+2+temp]);
                                }	
                                break;
                                
                          case 51:    // IP address leasetime                                
						  case 58:    // T1
                          case 59:    // T2
                                if (((unsigned char *)(data))[pos] == 51) {
                                    printk(KERN_INFO "%s: IP Address Lease Time: ", SQN_PRT_MODULE_NAME); 									
                                    printTime32(data + pos + 2);
								}

                                else if (((unsigned char *)(data))[pos] == 58) {
                                    printk(KERN_INFO "%s: Renew Time Value: ", SQN_PRT_MODULE_NAME);                                 
									printTime32(data + pos + 2);
								} 
								else if (((unsigned char *)(data))[pos] == 59) {
                                    // printk(KERN_INFO "%s: Option: (59) Rebinding Time Value", SQN_PRT_MODULE_NAME); 
									// printTime32(data + pos + 2);
								}                                                               
                                // printk(KERN_INFO "%s: Length:%d\n", SQN_PRT_MODULE_NAME,  (((unsigned char *)(data))[pos+1]) );  							        
                                
								break;

                        case 54:    // Server identifier                                
                                /*
								printk(KERN_INFO "%s: Server Identifier\n", SQN_PRT_MODULE_NAME);  
								// printk(KERN_INFO "%s: Length:%d\n", SQN_PRT_MODULE_NAME,  (((unsigned char *)(data))[pos+1]) );  							        
                                printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
                                for (temp=0; temp<4; temp++) {
                                    if (temp<3) 
                                            printk("%d.", ((unsigned char *)(data))[pos+2+temp]);							
                                        else
                                            printk("%d\n", ((unsigned char *)(data))[pos+2+temp]);
                                } 
                                */ 
                                break;

						  case 53:    // DHCP message type
								if ((((unsigned char *)(data))[pos+2]) == 1) {
									printk(KERN_INFO "%s: [DHCP Discover]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}														
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 2) {
									printk(KERN_INFO "%s: [DHCP Offer]\n", SQN_PRT_MODULE_NAME);  
				
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}											
				
									// Your IP
									printk(KERN_INFO "%s: Your IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=58; temp<62; temp++) {
										if (temp<61) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Server IP
									printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=62; temp<66; temp++) {
										if (temp<65) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 3) {
									printk(KERN_INFO "%s: [DHCP Request]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
										                                			
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 4) {
									printk(KERN_INFO "%s: [DHCP Decline]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
																											
									// Client IP
									printk(KERN_INFO "%s: Client IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=54; temp<58; temp++) {
										if (temp<57) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Your IP
									printk(KERN_INFO "%s: Your IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=58; temp<62; temp++) {
										if (temp<61) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Server IP
									printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=62; temp<66; temp++) {
										if (temp<65) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 5) {
									printk(KERN_INFO "%s: [DHCP Ack]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
																											
									// Client IP
									printk(KERN_INFO "%s: Client IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=54; temp<58; temp++) {
										if (temp<57) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Your IP
									printk(KERN_INFO "%s: Your IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=58; temp<62; temp++) {
										if (temp<61) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Server IP
									printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=62; temp<66; temp++) {
										if (temp<65) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 6) {
									printk(KERN_INFO "%s: [DHCP Nack]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
																											
									// Client IP
									printk(KERN_INFO "%s: Client IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=54; temp<58; temp++) {
										if (temp<57) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Your IP
									printk(KERN_INFO "%s: Your IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=58; temp<62; temp++) {
										if (temp<61) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Server IP
									printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=62; temp<66; temp++) {
										if (temp<65) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else if ((((unsigned char *)(data))[pos+2]) == 7) {
									printk(KERN_INFO "%s: [DHCP Release]\n", SQN_PRT_MODULE_NAME);  
									// Source IP
									printk(KERN_INFO "%s: Src IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=26; temp<30; temp++) {
										if (temp<29) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
									
									// Destination IP
									printk(KERN_INFO "%s: Dst IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=30; temp<34; temp++) {
										if (temp<33) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}					
																											
									// Client IP
									printk(KERN_INFO "%s: Client IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=54; temp<58; temp++) {
										if (temp<57) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Your IP
									printk(KERN_INFO "%s: Your IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=58; temp<62; temp++) {
										if (temp<61) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
				
									// Server IP
									printk(KERN_INFO "%s: Server IP: ", SQN_PRT_MODULE_NAME);  
									for (temp=62; temp<66; temp++) {
										if (temp<65) 
												printk("%d.", ((unsigned char *)(data))[temp]);							
											else
												printk("%d\n", ((unsigned char *)(data))[temp]);
									}	
													
									// Client MAC
									printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
									for (temp=70; temp<76; temp++) {
										if (temp<75) 
												printk("%02x:", ((unsigned char *)(data))[temp]);							
											else
												printk("%02x\n", ((unsigned char *)(data))[temp]);
									}	
								}
								else {
									printk(KERN_INFO "%s: Type: Unknown\n", SQN_PRT_MODULE_NAME);  
								}
								break;
                            
                            case 61:    // Client identifier
								printk(KERN_INFO "%s: Client identifier\n", SQN_PRT_MODULE_NAME);                               
                                printk(KERN_INFO "%s: Client MAC: ", SQN_PRT_MODULE_NAME);  
                                for (temp=0; temp<6; temp++) {
                                    if (temp<5) 
                                            printk("%02x:", ((unsigned char *)(data))[pos+3+temp]);							
                                        else
                                            printk("%02x\n", ((unsigned char *)(data))[pos+3+temp]);
                                }	                              
								break;
                                        
                            default:	
                                break;

						  } // Option case ]
						  						 
						  // This might go wrong if a mallformed packet is received.
						  // Maybe from a bogus server which is instructed to reply
						  // with invalid data and thus causing an exploit.
						  // My head hurts... but I think it's solved by the checking
						  // for pos<len_ at the begin of the while-loop.
						  if (data[pos] == 0) // padding
								pos++;
						  else
								pos+=data[pos + 1] + 2;

					} // while option ]
					
				}				
			} // DHCP ]

			if (len_ >= 34 && !bHandle) { // HTTP [				
				// IP: 0x0800, TCP: 0x06, port: 0x0050 (80)
				if (   (((unsigned char *)(data))[i+12] == 0x08) && 
					   (((unsigned char *)(data))[i+13] == 0x00) &&
					   (((unsigned char *)(data))[i+23] == 0x06) &&		
					   (			 
					     ((((unsigned char *)(data))[i+34] == 0x00) && (((unsigned char *)(data))[i+35] == 0x50)) || 
						 ((((unsigned char *)(data))[i+36] == 0x00) && (((unsigned char *)(data))[i+37] == 0x50))
					   )
				   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [HTTP] request, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					

					// Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d\n", ((unsigned char *)(data))[pos]);
					}			
				}				
			} // HTTP ]
			
			if (len_ >= 34 && !bHandle) { // DNS [				
				// IP: 0x0800, UDP: 0x11, port: 0x0035
				if (   (((unsigned char *)(data))[i+12] == 0x08) && 
					   (((unsigned char *)(data))[i+13] == 0x00) &&
					   (((unsigned char *)(data))[i+23] == 0x11) &&		
					   (			 
					     ((((unsigned char *)(data))[i+34] == 0x00) && (((unsigned char *)(data))[i+35] == 0x35)) || 
						 ((((unsigned char *)(data))[i+36] == 0x00) && (((unsigned char *)(data))[i+37] == 0x35))
					   )
				   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [DNS] query, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					

					// Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d\n", ((unsigned char *)(data))[pos]);
					}			
				}				
			} // DNS ]

			else if (len_ >= 34 && !bHandle) { // NTP [				
				// IP: 0x0800, UDP: 0x11, port: 0x007b
				if (   (((unsigned char *)(data))[i+12] == 0x08) && 
					   (((unsigned char *)(data))[i+13] == 0x00) &&
                       (((unsigned char *)(data))[i+23] == 0x11) &&					 
					   (((unsigned char *)(data))[i+34] == 0x00) &&
					   (((unsigned char *)(data))[i+35] == 0x7b)					  
                   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [NTP] Sync active, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					
                    
                    // Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d\n", ((unsigned char *)(data))[pos]);
					}			
				}				
			} // NTP ]
			
			if (len_ >= 12 && !bHandle) { // IPv6 [				
				// IPv6: 0x86DD, UDP: 0x11
				if (   (((unsigned char *)(data))[i+12] == 0x86) && 
					   (((unsigned char *)(data))[i+13] == 0xDD)                        
                   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [IPv6] Network packets, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Dst MAC: ");  
					for (pos=0; pos<6; pos++) {
						if (pos<5) 
								printk("%d:", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					
                    
                    // Destination IP
					printk("Src MAC: ");  
					for (pos=6; pos<12; pos++) {
						if (pos<11) 
								printk("%d:", ((unsigned char *)(data))[pos]);							
							else
								printk("%d\n", ((unsigned char *)(data))[pos]);
					}								
				}				
			} // IPv6 ]

			if (len_ >= 34 && !bHandle) { // Unknown UDP [				
				// IP: 0x0800, UDP: 0x11
				if (   (((unsigned char *)(data))[i+12] == 0x08) && 
					   (((unsigned char *)(data))[i+13] == 0x00) &&
                       (((unsigned char *)(data))[i+23] == 0x11) 					 			  
                   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [UDP] Network packets, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					
                    
                    // Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}			

					printk("Port: %d%d\n",  ((unsigned char *)(data))[i+34], ((unsigned char *)(data))[i+35]);  					  
				}				
			} // Unknown ]

			if (len_ >= 34 && !bHandle) { // Unknown TCP [				
				// IP: 0x0800, TCP: 0x06
				if (   (((unsigned char *)(data))[i+12] == 0x08) && 
					   (((unsigned char *)(data))[i+13] == 0x00) &&
                       (((unsigned char *)(data))[i+23] == 0x06) 					 			  
                   ) {
					bHandle = 1;
					printk(KERN_INFO "%s: [TCP] Network packets, ", SQN_PRT_MODULE_NAME);					

					// Source IP
					printk("Src IP: ");  
					for (pos=26; pos<30; pos++) {
						if (pos<29) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}					
                    
                    // Destination IP
					printk("Dst IP: ");  
					for (pos=30; pos<34; pos++) {
						if (pos<33) 
								printk("%d.", ((unsigned char *)(data))[pos]);							
							else
								printk("%d, ", ((unsigned char *)(data))[pos]);
					}			

					printk("Port: %d%d\n",  ((unsigned char *)(data))[i+34], ((unsigned char *)(data))[i+35]);  					  
				}				
			} // Unknown ]

            // Andrew 0903
			// printk(KERN_INFO "%s: %s: %04x ", SQN_PRT_MODULE_NAME, (prefix), i);            
        } // if (i % width == 0)

        // Andrew 0903
		// printk("%02x ", ((unsigned char *)(data))[i++]);
		if ((i % width == 0) || (i == len_))
           printk("\n");
	}	
}


#define MAX_DUMP_LEN 48

void sqn_pr_info_dump_rawdata(char *prefix, unsigned char *data, unsigned int len) {
   	unsigned int i = 0;
	unsigned int width = 16;
	unsigned int len_ = (unsigned int)(len);

	if (len_ > MAX_DUMP_LEN) {
		len_ = MAX_DUMP_LEN;
	}

    // sequans_xxx: RX PDU: 0000 ff ff ff ff ff ff 00 1e 90 21 0b d4 08 00 45 00 
	while (i < len_) {
		if (i % width == 0)
			printk(KERN_INFO "%s: %s: %04x ", SQN_PRT_MODULE_NAME, (prefix), i);                   
       
		printk("%02x ", ((unsigned char *)(data))[i++]);

		if ((i % width == 0) || (i == len_))
           printk("\n");
	}	
}

int sqn_filter_packet_check(char *prefix, unsigned char *data, unsigned int len) {
   	unsigned int i = 0, pos = 0, temp = 0;
	unsigned int width = 16;
	unsigned int len_ = (unsigned int)(len);

    char buf[LARGESTRING];

	int bHandle = 0, bFilter = 0;
		
    if (i < len_) {   		
		// Unblocked list:
		if (len_ >= 40 && !bHandle) { // ARP [						
			if ( (((unsigned char *)(data))[i+12] == 0x08) && 
				 (((unsigned char *)(data))[i+13] == 0x06) ) {				
				bHandle = 1;
				bFilter = 0;
			}				
		} // ARP ]

		if (len_ >= 34 && !bHandle) { // ICMP [	
			// IP: 0x0800
			if ( (((unsigned char *)(data))[i+12] == 0x08) && 
				 (((unsigned char *)(data))[i+13] == 0x00) &&
				 (((unsigned char *)(data))[i+23] == 0x01) 
			   ) {			
				bHandle = 1;	
				bFilter = 0;
			}				
		} // ICMP ]

		if (len_ >= 300 && !bHandle) { // DHCP [		
			// IP: 0x0800, UDP: 0x11, port: 0x0044, 0x0043
			if ( (((unsigned char *)(data))[i+12] == 0x08) && 
				 (((unsigned char *)(data))[i+13] == 0x00) &&
				 (((unsigned char *)(data))[i+23] == 0x11) &&

				 (
				   (((unsigned char *)(data))[i+34] == 0x00) &&
				   ((((unsigned char *)(data))[i+35] == 0x44) || (((unsigned char *)(data))[i+35] == 0x43)) &&
				   (((unsigned char *)(data))[i+36] == 0x00) &&
				   ((((unsigned char *)(data))[i+37] == 0x43) || (((unsigned char *)(data))[i+37] == 0x44))
				 ) 
			   ) {
			
				bHandle = 1;
				bFilter = 0;
			}
		} // DHCP ]
			 		
		if (len_ >= 34 && !bHandle) { // HTTP [				
			// IP: 0x0800, TCP: 0x06, port: 0x0050 (80)
			if (   (((unsigned char *)(data))[i+12] == 0x08) && 
				   (((unsigned char *)(data))[i+13] == 0x00) &&
				   (((unsigned char *)(data))[i+23] == 0x06) &&		
				   (			 
					 ((((unsigned char *)(data))[i+34] == 0x00) && (((unsigned char *)(data))[i+35] == 0x50)) || 
					 ((((unsigned char *)(data))[i+36] == 0x00) && (((unsigned char *)(data))[i+37] == 0x50))
				   )
			   ) {
				sqn_pr_info("Drop HTTP packets len:%d\n", len_);
				bHandle = 1;
				bFilter = 1;			
			}				
		} // HTTP ]

		if (len_ >= 34 && !bHandle) { // DNS [				
			// IP: 0x0800, UDP: 0x11, port: 0x0035
			if (   (((unsigned char *)(data))[i+12] == 0x08) && 
				   (((unsigned char *)(data))[i+13] == 0x00) &&
				   (((unsigned char *)(data))[i+23] == 0x11) &&		
				   (			 
					 ((((unsigned char *)(data))[i+34] == 0x00) && (((unsigned char *)(data))[i+35] == 0x35)) || 
					 ((((unsigned char *)(data))[i+36] == 0x00) && (((unsigned char *)(data))[i+37] == 0x35))
				   )
			   ) {
				bHandle = 1;
				bFilter = 0;
			}
		} // DNS ]

		else if (len_ >= 34 && !bHandle) { // NTP [				
			// IP: 0x0800, UDP: 0x11, port: 0x007b
			if (   (((unsigned char *)(data))[i+12] == 0x08) && 
				   (((unsigned char *)(data))[i+13] == 0x00) &&
				   (((unsigned char *)(data))[i+23] == 0x11) &&					 
				   (((unsigned char *)(data))[i+34] == 0x00) &&
				   (((unsigned char *)(data))[i+35] == 0x7b)					  
			   ) {
				sqn_pr_info("Drop NTP packets len:%d\n", len_);
				bHandle = 1;
				bFilter = 1;
			}				
		} // NTP ]

		// Block list:
		if (len_ >= 12 && !bHandle) { // IPv6 [				
				// IPv6: 0x86DD, UDP: 0x11
				if (   (((unsigned char *)(data))[i+12] == 0x86) && 
					   (((unsigned char *)(data))[i+13] == 0xDD)                        
                   ) {
					sqn_pr_info("Drop IPv6 packets len:%d\n", len_);
					bHandle = 1;				
					bFilter = 1;
				}				
		} // IPv6 ]

		if (len_ >= 34 && !bHandle) { // Unknown UDP [				
			// IP: 0x0800, UDP: 0x11
			if (   (((unsigned char *)(data))[i+12] == 0x08) && 
				   (((unsigned char *)(data))[i+13] == 0x00) &&
				   (((unsigned char *)(data))[i+23] == 0x11) 					 			  
			   ) {
				sqn_pr_info("Drop UDP packets len:%d\n", len_);
				bHandle = 1;
				bFilter = 1;			
			}				
		} // Unknown UDP ]

		if (len_ >= 34 && !bHandle) { // Unknown TCP [				
			// IP: 0x0800, TCP: 0x06
			if (   (((unsigned char *)(data))[i+12] == 0x08) && 
				   (((unsigned char *)(data))[i+13] == 0x00) &&
				   (((unsigned char *)(data))[i+23] == 0x06) 					 			  
			   ) {
				sqn_pr_info("Drop TCP packets len:%d\n", len_);
				bHandle = 1;
				bFilter = 1;
			}				
		} // Unknown TCP ]
	}
	
	return bFilter;	
}

// print the data as a 32bits time-value
void printTime32(u_char *data) {
      int t = (data[0] << 24) + (data[1] << 16) + (data[2] <<8 ) + data[3];
      printk("%d (", t);
      if (t > SPERW) { printk("%dw", t / (SPERW)); t %= SPERW; }
      if (t > SPERD) { printk("%dd", t / (SPERD)); t %= SPERD; }
      if (t > SPERH) { printk("%dh", t / (SPERH)); t %= SPERH; }
      if (t > SPERM) { printk("%dm", t / (SPERM)); t %= SPERM; }
      if (t > 0) printk("%ds", t);
      printk(")");
}
