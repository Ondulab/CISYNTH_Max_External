/*
    udpReceive.c - receive data from Ondulab CIS devices

    this object has one inlet and one outlet
    it decodes formatted image lines from CIS in udp
    it sends to MAX outlet ARGB array

    https://github.com/CNMAT/CNMAT-Externs source code is a great source of inspiration
    https://github.com/siteswapjuggler/smartball-externals source code of inspiration
    https://github.com/Ondulab/SSS_CIS source code of CIS device
*/

#include "cis_receive.h"

// Ensure that you have global symbols for your selectors
static t_symbol *s_LineLow_R;
static t_symbol *s_LineLow_G;
static t_symbol *s_LineLow_B;
static t_symbol *s_Line_R;
static t_symbol *s_Line_G;
static t_symbol *s_Line_B;
static t_symbol *s_IMU_Ax;
static t_symbol *s_IMU_Ay;
static t_symbol *s_IMU_Az;
static t_symbol *s_IMU_Gx;
static t_symbol *s_IMU_Gy;
static t_symbol *s_IMU_Gz;
static t_symbol *s_HID_B1;
static t_symbol *s_HID_B2;
static t_symbol *s_HID_B3;

void cisReceive_readStartupInfo(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readButtonData(t_cisReceive *x, void *data, uint16_t length);
void send_data_to_outlet(void* object, t_symbol* sel, int argc, t_atom* argv);
void cisReceive_clock_tick(t_cisReceive *x);

//---------------------------------------------------------------------------------------------------------------------------------------------------------
//  INSTANCE DECLARATION
//---------------------------------------------------------------------------------------------------------------------------------------------------------

t_class *cisReceiveclass;        // global pointer to the object class - so max can reference the object

void ext_main(void *r)
{
    t_class *c;                                                                                                            // pointer to a class type
    c = class_new("cis_receive", (method)cisReceive_new, (method)cisReceive_free, sizeof(t_cisReceive), 0L, A_GIMME, 0);     // class_new() loads our external's class into Max's memory so it can be used in a patch
    class_register(CLASS_BOX, c);                                                                                        // register to CLASS_BOX type for max environment
    cisReceiveclass = c;
    post("cis_receive v1.02 - 10.10.2024");
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// CONSTRUCTION, DESTRUCTION
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void *cisReceive_new(t_symbol *s, long argc, t_atom *argv)
{
    t_cisReceive *x = (t_cisReceive *)object_alloc(cisReceiveclass);

    post("new udpReceive");
    
    // HANDLE ARGUMENTS
    if (argc == 0) {
        x->port = DEFAULT_PORT;
        x->multicast = DEFAULT_MULTI;
        post("Default port: %d", DEFAULT_PORT);
    }
    else {
    ARG_FAULT:
        object_error((t_object*)x, "Cannot start with those arguments ");
        goto NOT_VALID;
    }
    
    x->clock = clock_new(x, (method)cisReceive_clock_tick);
    clock_delay(x->clock, 1000); // Starts after an initial delay of 40 ms
    
    // Initialize the image buffer
    x->image_buffer_R = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    x->image_buffer_G = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    x->image_buffer_B = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    
    // After allocating the buffers
    if (!x->image_buffer_R || !x->image_buffer_G || !x->image_buffer_B) {
        object_error((t_object*)x, "Allocation error for image_buffer");
        // Free memory here if any allocation succeeded before failure
        if (x->image_buffer_R) sysmem_freeptr(x->image_buffer_R);
        if (x->image_buffer_G) sysmem_freeptr(x->image_buffer_G);
        if (x->image_buffer_B) sysmem_freeptr(x->image_buffer_B);
        return NULL; // Return NULL to indicate object creation failure
    }
    
    x->IMU_Ax = 0.0f;
    x->IMU_Ay = 0.0f;
    x->IMU_Az = 0.0f;
    x->IMU_Gx = 0.0f;
    x->IMU_Gy = 0.0f;
    x->IMU_Gz = 0.0f;
    
    x->HID_B1 = 0;
    x->HID_B2 = 0;
    x->HID_B3 = 0;
    
    x->atom_buffer_R = (t_atom *)sysmem_newptr(CIS_PIXELS_NB * sizeof(t_atom));
    x->atom_buffer_G = (t_atom *)sysmem_newptr(CIS_PIXELS_NB * sizeof(t_atom));
    x->atom_buffer_B = (t_atom *)sysmem_newptr(CIS_PIXELS_NB * sizeof(t_atom));
    
    x->atom_IMU_Ax = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_IMU_Ay = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_IMU_Az = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_IMU_Gx = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_IMU_Gy = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_IMU_Gz = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_HID_B1 = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_HID_B2 = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    x->atom_HID_B3 = (t_atom *)sysmem_newptr(1 * sizeof(t_atom));
    
    // Check for successful allocation of all buffers
    if (!x->atom_buffer_R || !x->atom_buffer_G || !x->atom_buffer_B ||
        !x->atom_IMU_Ax || !x->atom_IMU_Ay || !x->atom_IMU_Az ||
        !x->atom_IMU_Gx || !x->atom_IMU_Gy || !x->atom_IMU_Gz ||
        !x->atom_HID_B1 || !x->atom_HID_B2 || !x->atom_HID_B3) {
        // Free allocated resources
        if (x->atom_buffer_R) sysmem_freeptr(x->atom_buffer_R);
        if (x->atom_buffer_G) sysmem_freeptr(x->atom_buffer_G);
        if (x->atom_buffer_B) sysmem_freeptr(x->atom_buffer_B);
        // Continue for other allocations...
        object_error((t_object*)x, "Allocation error for atom_buffer");
        return NULL;
    }

    s_LineLow_R = gensym("lineLow_R");
    s_LineLow_G = gensym("lineLow_G");
    s_LineLow_B = gensym("lineLow_B");
    
    s_Line_R = gensym("line_R");
    s_Line_G = gensym("line_G");
    s_Line_B = gensym("line_B");
    
    s_IMU_Ax = gensym("IMU_Ax");
    s_IMU_Ay = gensym("IMU_Ay");
    s_IMU_Az = gensym("IMU_Az");
    
    s_IMU_Gx = gensym("IMU_Gx");
    s_IMU_Gy = gensym("IMU_Gy");
    s_IMU_Gz = gensym("IMU_Gz");
    
    s_HID_B1 = gensym("HID_B1");
    s_HID_B2 = gensym("HID_B2");
    s_HID_B3 = gensym("HID_B3");

    // HANDLE SOCKET INITIATION
    if (syssock_set(x) < 0) goto NOT_VALID;
    
    // CREATE OUTLETS
    x->outlet_Image = outlet_new(x, NULL);
    x->outlet_LowImage = outlet_new(x, NULL);
    x->outlet_IMU = outlet_new(x, NULL);
    x->outlet_HID = outlet_new(x, NULL);
    
    return(x);

NOT_VALID:
    x = NULL;
    return(x);
}

void cisReceive_free(t_cisReceive *x)
{
    if (x->listener)
    {
        x->listening = false;
        if (x->fd) {
            syssock_dropmulticast(x->fd, x->multicast);
            syssock_close(x->fd);
            x->fd = 0; // Ensure the descriptor is reset
        }
        systhread_join(x->listener, NULL);
        x->listener = NULL;
    }
    
    // Free the image buffer
    if (x->image_buffer_R) {
        sysmem_freeptr(x->image_buffer_R);
        x->image_buffer_R = NULL; // Reset the pointer after freeing
    }
    if (x->image_buffer_G) {
        sysmem_freeptr(x->image_buffer_G);
        x->image_buffer_G = NULL; // Reset the pointer after freeing
    }
    if (x->image_buffer_B) {
        sysmem_freeptr(x->image_buffer_B);
        x->image_buffer_B = NULL; // Reset the pointer after freeing
    }

    // Added to free the t_atom buffers if they were allocated
    if (x->atom_buffer_R) {
        sysmem_freeptr(x->atom_buffer_R);
        x->atom_buffer_R = NULL; // Reset the pointer after freeing
    }
    if (x->atom_buffer_G) {
        sysmem_freeptr(x->atom_buffer_G);
        x->atom_buffer_G = NULL; // Reset the pointer after freeing
    }
    if (x->atom_buffer_B) {
        sysmem_freeptr(x->atom_buffer_B);
        x->atom_buffer_B = NULL; // Reset the pointer after freeing
    }

    clock_free(x->clock);
}

void cisReceiveassist(t_cisReceive *x, void *b, long m, long a, char *s) {
    if (m == ASSIST_INLET)
        ;
    else
        sprintf(s, "Smartball feedbacks");
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// MESSAGE LISTENER
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void cisReceive_readStartupInfo(t_cisReceive *x, void *data, uint16_t length) {
    if (length == sizeof(struct packet_StartupInfo)) {
        struct packet_StartupInfo *packet = (struct packet_StartupInfo *)data;

        // Store packet ID and version info in x structure
        x->startup_packet_id = packet->packet_id;
        strncpy(x->version_info, (const char *)packet->version_info, sizeof(x->version_info) - 1);
        x->version_info[sizeof(x->version_info) - 1] = '\0'; // Ensure it's a valid string

        // Post a message or perform other actions based on the startup info received
        post("Startup Packet ID: %u, Version Info: %s", packet->packet_id, x->version_info);
    } else {
        object_error((t_object*)x, "Invalid Startup Info packet length.");
    }
}

void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length) {
    static uint32_t curr_line_id = 0;
    static bool received_fragments_R[UDP_NB_PACKET_PER_LINE] = {0}; //todo make smart
    static bool received_fragments_G[UDP_NB_PACKET_PER_LINE] = {0};
    static bool received_fragments_B[UDP_NB_PACKET_PER_LINE] = {0};
    static uint32_t offset = 0;
    static uint8_t color = 0;
    
    if (length == sizeof(struct packet_Image)) { //448
        struct packet_Image *packet = (struct packet_Image *)data;

        if (curr_line_id != packet->line_id)
        {
            curr_line_id = packet->line_id;
            memset(received_fragments_R, 0, packet->total_fragments * sizeof(bool));
            memset(received_fragments_G, 0, packet->total_fragments * sizeof(bool));
            memset(received_fragments_B, 0, packet->total_fragments * sizeof(bool));
        }
        
        offset = packet->fragment_id * packet->fragment_size;
        
        for (uint32_t i = 0; i < packet->fragment_size; i++) {
            x->image_buffer_R[offset + i] = packet->imageData_R[i];
            x->image_buffer_G[offset + i] = packet->imageData_G[i];
            x->image_buffer_B[offset + i] = packet->imageData_B[i];
        }
        
        received_fragments_R[packet->fragment_id] = TRUE;
        received_fragments_G[packet->fragment_id] = TRUE;
        received_fragments_B[packet->fragment_id] = TRUE;

        x->line_complete = TRUE; // Start by assuming the line is complete

        for (int i = 0; i < packet->total_fragments; i++) {
            if (!received_fragments_R[i] || !received_fragments_G[i] || !received_fragments_B[i]) {
                x->line_complete = FALSE; // As soon as a missing fragment is found, mark the line as incomplete
                break; // No need to check other fragments
            }
        }
    }
}

void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length) {
    
    if (length == sizeof(struct packet_IMU)) {
        struct packet_IMU *packet = (struct packet_IMU *)data;
        
        //post("ID: %d", packet->packet_id);
        
        //post("acc X: %f", packet->acc[0]);
        //post("acc Y: %f", packet->acc[1]);
        //post("acc Z: %f", packet->acc[2]);
        
        //post("gyro X: %f", packet->gyro[0]);
        //post("gyro Y: %f", packet->gyro[1]);
        //post("gyro Z: %f", packet->gyro[2]);
        
        x->IMU_Ax = packet->acc[0];
        x->IMU_Ay = packet->acc[1];
        x->IMU_Az = packet->acc[2];
        
        x->IMU_Gx = packet->gyro[0];
        x->IMU_Gy = packet->gyro[1];
        x->IMU_Gz = packet->gyro[2];
    }
}

void cisReceive_readButtonData(t_cisReceive *x, void *data, uint16_t length) {
    if (length == sizeof(struct packet_Button)) {
        struct packet_Button *packet = (struct packet_Button *)data;

        // Update button states in x structure
        // Update button states in x structure
        switch (packet->button_id)
        {
            case SW1:
                x->HID_B1 = packet->button_state.state;
                break;
            case SW2:
                x->HID_B2 = packet->button_state.state;
                break;
            case SW3:
                x->HID_B3 = packet->button_state.state;
                break;
            default:
                break;
        }

        // Post button states or trigger actions
        //post("HID Packet ID: %u, Button A: %u, Button B: %u, Button C: %u", packet->packet_id, packet->button_A, packet->button_B, packet->button_C);

        // Update atoms for sending to outlets (already implemented in cisReceive_read function)
    } else {
        object_error((t_object*)x, "Invalid HID Data packet length.");
    }
}
    
void cisReceive_read(t_cisReceive *x) {
    uint8_t msgbuf[sizeof(struct packet_Image)];
    uint32_t nbytes;
    socklen_t addrlen = sizeof(x->addr);

    while (x->listening) {
        nbytes = (uint32_t)recvfrom(x->fd, msgbuf, sizeof(struct packet_Image), 0, (struct sockaddr *) &x->addr, &addrlen);

        if (nbytes < 0) {
            // Error handling for recvfrom
            object_error((t_object*)x, "Error receiving data: %d", errno);
            return; // Continuing with the loop might be an option, or you could choose to exit depending on your use case
        }
        
        switch (msgbuf[0]) {
            case STARTUP_INFO_HEADER:
                if (nbytes >= sizeof(struct packet_StartupInfo)) {
                    cisReceive_readStartupInfo(x, msgbuf, nbytes);
                } else {
                    object_error((t_object*)x, "Malformed STARTUP_INFO_HEADER packet.");
                }
                break;
            case IMAGE_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_Image)) {
                    cisReceive_readImageData(x, msgbuf, nbytes);
                } else {
                    object_error((t_object*)x, "Malformed IMAGE_DATA_HEADER packet.");
                }
                break;
            case IMU_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_IMU)) {
                    cisReceive_readImuData(x, msgbuf, nbytes);
                    atom_setfloat(x->atom_IMU_Ax, x->IMU_Ax);
                    atom_setfloat(x->atom_IMU_Ay, x->IMU_Ay);
                    atom_setfloat(x->atom_IMU_Az, x->IMU_Az);
                    atom_setfloat(x->atom_IMU_Gx, x->IMU_Gx);
                    atom_setfloat(x->atom_IMU_Gy, x->IMU_Gy);
                    atom_setfloat(x->atom_IMU_Gz, x->IMU_Gz);
                    send_data_to_outlet(x, s_IMU_Ax, 1, x->atom_IMU_Ax);
                    send_data_to_outlet(x, s_IMU_Ay, 1, x->atom_IMU_Ay);
                    send_data_to_outlet(x, s_IMU_Az, 1, x->atom_IMU_Az);
                    send_data_to_outlet(x, s_IMU_Gx, 1, x->atom_IMU_Gx);
                    send_data_to_outlet(x, s_IMU_Gy, 1, x->atom_IMU_Gy);
                    send_data_to_outlet(x, s_IMU_Gz, 1, x->atom_IMU_Gz);
                } else {
                    object_error((t_object*)x, "Malformed IMU_DATA_HEADER packet.");
                }
                break;
            case BUTTON_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_Button)) {
                    cisReceive_readButtonData(x, msgbuf, nbytes);
                    atom_setlong(x->atom_HID_B1, (long)x->HID_B1);
                    atom_setlong(x->atom_HID_B2, (long)x->HID_B2);
                    atom_setlong(x->atom_HID_B3, (long)x->HID_B3);
                    send_data_to_outlet(x, s_HID_B1, 1, x->atom_HID_B1);
                    send_data_to_outlet(x, s_HID_B2, 1, x->atom_HID_B2);
                    send_data_to_outlet(x, s_HID_B3, 1, x->atom_HID_B3);
                } else {
                    object_error((t_object*)x, "Malformed HID_DATA_HEADER packet.");
                }
                break;
            default:
                object_error((t_object*)x, "Unknown packet header.");
                return;
        }

        if (x->line_complete == TRUE) {
            for (int i = 0; i < CIS_PIXELS_NB; i++) {
                atom_setlong(&(x->atom_buffer_R[i]), (long)x->image_buffer_R[i]);
                atom_setlong(&(x->atom_buffer_G[i]), (long)x->image_buffer_G[i]);
                atom_setlong(&(x->atom_buffer_B[i]), (long)x->image_buffer_B[i]);
            }
            
            // Send the atom list
            send_data_to_outlet(x, s_Line_R, CIS_PIXELS_NB, x->atom_buffer_R);
            send_data_to_outlet(x, s_Line_G, CIS_PIXELS_NB, x->atom_buffer_G);
            send_data_to_outlet(x, s_Line_B, CIS_PIXELS_NB, x->atom_buffer_B);
        }
    }
}

void cisReceive_clock_tick(t_cisReceive *x) {
    defer_low(x, (method)send_data_to_outlet, s_LineLow_R, CIS_PIXELS_NB, x->atom_buffer_R);
    defer_low(x, (method)send_data_to_outlet, s_LineLow_G, CIS_PIXELS_NB, x->atom_buffer_G);
    defer_low(x, (method)send_data_to_outlet, s_LineLow_B, CIS_PIXELS_NB, x->atom_buffer_B);

    // Schedule the next tick in 10 ms
    clock_delay(x->clock, 10);
}

void send_data_to_outlet(void* object, t_symbol* sel, int argc, t_atom* argv) {
    t_cisReceive *x = (t_cisReceive *)object;
    
    if(sel == s_LineLow_R) {
        outlet_anything(x->outlet_LowImage, s_LineLow_R, argc, argv);
    }
    if(sel == s_LineLow_G) {
        outlet_anything(x->outlet_LowImage, s_LineLow_G, argc, argv);
    }
    if(sel == s_LineLow_B) {
        outlet_anything(x->outlet_LowImage, s_LineLow_B, argc, argv);
    }

    if(sel == s_Line_R) {
        outlet_anything(x->outlet_Image, s_Line_R, argc, argv);
    }
    if(sel == s_Line_G) {
        outlet_anything(x->outlet_Image, s_Line_G, argc, argv);
    }
    if(sel == s_Line_B) {
        outlet_anything(x->outlet_Image, s_Line_B, argc, argv);
    }

    if(sel == s_IMU_Ax) {
        outlet_anything(x->outlet_IMU, s_IMU_Ax, argc, argv);
    }
    if(sel == s_IMU_Ay) {
        outlet_anything(x->outlet_IMU, s_IMU_Ay, argc, argv);
    }
    if(sel == s_IMU_Az) {
        outlet_anything(x->outlet_IMU, s_IMU_Az, argc, argv);
    }
    
    if(sel == s_IMU_Gx) {
        outlet_anything(x->outlet_IMU, s_IMU_Gx, argc, argv);
    }
    if(sel == s_IMU_Gy) {
        outlet_anything(x->outlet_IMU, s_IMU_Gy, argc, argv);
    }
    if(sel == s_IMU_Gz) {
        outlet_anything(x->outlet_IMU, s_IMU_Gz, argc, argv);
    }

    if(sel == s_HID_B1) {
        outlet_anything(x->outlet_HID, s_HID_B1, argc, argv);
    }
    if(sel == s_HID_B2) {
        outlet_anything(x->outlet_HID, s_HID_B2, argc, argv);
    }
    if(sel == s_HID_B3) {
        outlet_anything(x->outlet_HID, s_HID_B3, argc, argv);
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// SYSSOCK EXTENSIONS
//---------------------------------------------------------------------------------------------------------------------------------------------------------

int syssock_set(t_cisReceive *x) {
    if (syssock_init()) {
        object_error((t_object*)x, "Cannot start socket API");
        return -1;
    }

    x->fd = syssock_socket(AF_INET, SOCK_DGRAM, 0);
    if (x->fd < 0) {
        object_error((t_object*)x, "Cannot start UDP socket");
        return -1;
    }

    if (syssock_reuseaddr(x->fd)) {
        object_error((t_object*)x, "Cannot set socket REUSEADDR option");
        return -1;
    }

    memset(&x->addr, 0, sizeof(x->addr));
    x->addr.sin_family = AF_INET;
    x->addr.sin_addr.s_addr = (in_addr_t)syssock_htonl(INADDR_ANY);
    x->addr.sin_port = htons(x->port);

    if (syssock_bind(x->fd, &x->addr)) {
        object_error((t_object*)x, "Cannot bind to port %d", x->port);
        return -1;
    } else {
        post("Socket bound to port %d", x->port); // Confirmation of binding to port
    }

    x->listening = true;
    systhread_create((method)cisReceive_read, x, 0, 32768, 0, &(x->listener));

    return 0;
}

int syssock_addmulticast(t_syssocket sockfd, char* ip) {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = syssock_inet_addr(ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    return syssock_setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
}

int syssock_dropmulticast(t_syssocket sockfd, char* ip) {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = syssock_inet_addr(ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    return syssock_setsockopt(sockfd, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// TCP handle
//---------------------------------------------------------------------------------------------------------------------------------------------------------

int tcp_connect(const char* server_ip, int server_port) {
    int sockfd;
    struct sockaddr_in server_addr;

    // Crée un socket TCP
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Erreur lors de la création du socket");
        return -1;
    }

    // Configure l'adresse du serveur
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
        perror("Adresse non valide ou non supportée");
        return -1;
    }

    // Connecte au serveur
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Erreur lors de la connexion");
        return -1;
    }

    return sockfd; // retourne le descripteur du socket
}

ssize_t tcp_send(int sockfd, const void* data, size_t length) {
    return send(sockfd, data, length, 0); // Envoi des données via TCP
}

void send_led_command(int sockfd) {
    struct led_State cmd = {1, 5, 1, 500, 500, 0.1, 0.1};
    
    if (tcp_send(sockfd, &cmd, sizeof(cmd)) < 0) {
        perror("Erreur lors de l'envoi de la commande LED");
    }
}

//int sockfd = tcp_connect("192.168.1.100", 12345); // Adresse IP et port du serveur
//if (sockfd >= 0) {
//    send_led_command(sockfd); // Envoie la commande LED
//    close(sockfd); // Ferme la connexion après l'envoi
//}
