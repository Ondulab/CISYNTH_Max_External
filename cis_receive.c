/*
    udpReceive.c - receive data from Ondulab CIS devices

    This object has one inlet and one outlet.
    It decodes formatted image lines from CIS in UDP
    and sends to MAX outlet ARGB array.

    https://github.com/CNMAT/CNMAT-Externs source code is a great source of inspiration.
    https://github.com/siteswapjuggler/smartball-externals source code of inspiration.
    https://github.com/Ondulab/SSS_CIS source code of CIS device.
*/

#include "cis_receive.h"
#include "ext_systhread.h"
#include "ext.h"
#include "ext_obex.h"

// Macros for memory management
#define FREE_AND_NULL(ptr) if (ptr) { sysmem_freeptr(ptr); ptr = NULL; }

#define DEFINE_LED_ATTR(led, attr_name, min, max) \
    CLASS_ATTR_LONG(c, "led" #led "_" #attr_name, 0, t_cisReceive, dummy_attr); \
    CLASS_ATTR_ACCESSORS(c, "led" #led "_" #attr_name, (method)led_attr_get, (method)led_attr_set); \
    CLASS_ATTR_FILTER_CLIP(c, "led" #led "_" #attr_name, min, max);

// Ensure that you have global symbols for your selectors
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

// Prototypes of helper functions
void *allocate_and_check(t_object *x, uint32_t size);
void cisReceive_cleanup(t_cisReceive *x);
void cisReceiveassist(t_cisReceive *x, void *b, long m, long a, char *s);
void cisReceive_readStartupInfo(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readButtonData(t_cisReceive *x, void *data, uint16_t length);
void send_data_to_outlet(void* object, t_symbol* sel, int argc, t_atom* argv);
void cisReceive_ledCommand(t_cisReceive *x, t_symbol *s, long argc, t_atom *argv);
t_max_err led_attr_get(t_cisReceive *x, void *attr, long *ac, t_atom **av);
t_max_err led_attr_set(t_cisReceive *x, void *attr, long ac, t_atom *av);

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// INSTANCE DECLARATION
//---------------------------------------------------------------------------------------------------------------------------------------------------------

t_class *cisReceiveclass;        // global pointer to the object class - so max can reference the object

void ext_main(void *r)
{
    t_class *c;
    c = class_new("cis_receive", (method)cisReceive_new, (method)cisReceive_free, sizeof(t_cisReceive), 0L, A_GIMME, 0);
    class_addmethod(c, (method)cisReceive_ledCommand, "anything", A_GIMME, 0); // Register "anything" message handler
    class_addmethod(c, (method)cisReceiveassist, "assist", A_CANT, 0);

    // Define LED attributes for each LED
    DEFINE_LED_ATTR(1, brightness_1, 0, 100);
    DEFINE_LED_ATTR(1, time_1, 0, 10000);
    DEFINE_LED_ATTR(1, glide_1, 0, 100);
    DEFINE_LED_ATTR(1, brightness_2, 0, 100);
    DEFINE_LED_ATTR(1, time_2, 0, 10000);
    DEFINE_LED_ATTR(1, glide_2, 0, 100);
    DEFINE_LED_ATTR(1, blink_count, 0, 100);

    DEFINE_LED_ATTR(2, brightness_1, 0, 100);
    DEFINE_LED_ATTR(2, time_1, 0, 10000);
    DEFINE_LED_ATTR(2, glide_1, 0, 100);
    DEFINE_LED_ATTR(2, brightness_2, 0, 100);
    DEFINE_LED_ATTR(2, time_2, 0, 10000);
    DEFINE_LED_ATTR(2, glide_2, 0, 100);
    DEFINE_LED_ATTR(2, blink_count, 0, 100);

    DEFINE_LED_ATTR(3, brightness_1, 0, 100);
    DEFINE_LED_ATTR(3, time_1, 0, 10000);
    DEFINE_LED_ATTR(3, glide_1, 0, 100);
    DEFINE_LED_ATTR(3, brightness_2, 0, 100);
    DEFINE_LED_ATTR(3, time_2, 0, 10000);
    DEFINE_LED_ATTR(3, glide_2, 0, 100);
    DEFINE_LED_ATTR(3, blink_count, 0, 100);

    class_register(CLASS_BOX, c);
    cisReceiveclass = c;

    // Initialize symbols globally
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

    post("cis_receive v2.00 - 13.10.2024");
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// MEMORY ALLOCATION HELPERS
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void *allocate_and_check(t_object *x, uint32_t size)
{
    void *ptr = sysmem_newptr(size);
    if (!ptr)
    {
        object_error(x, "Allocation error for buffer");
    }
    return ptr;
}

void cisReceive_cleanup(t_cisReceive *x)
{
    FREE_AND_NULL(x->image_buffer_R);
    FREE_AND_NULL(x->image_buffer_G);
    FREE_AND_NULL(x->image_buffer_B);
    FREE_AND_NULL(x->atom_buffer_R);
    FREE_AND_NULL(x->atom_buffer_G);
    FREE_AND_NULL(x->atom_buffer_B);
    FREE_AND_NULL(x->atom_IMU_Ax);
    FREE_AND_NULL(x->atom_IMU_Ay);
    FREE_AND_NULL(x->atom_IMU_Az);
    FREE_AND_NULL(x->atom_IMU_Gx);
    FREE_AND_NULL(x->atom_IMU_Gy);
    FREE_AND_NULL(x->atom_IMU_Gz);
    FREE_AND_NULL(x->atom_HID_B1);
    FREE_AND_NULL(x->atom_HID_B2);
    FREE_AND_NULL(x->atom_HID_B3);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// CONSTRUCTION, DESTRUCTION
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void *cisReceive_new(t_symbol *s, long argc, t_atom *argv)
{
    t_cisReceive *x = (t_cisReceive *)object_alloc(cisReceiveclass);

    post("new udpReceive");

    x->port = DEFAULT_PORT;
    x->multicast = DEFAULT_MULTI;
    
    // Initialize the image buffers
    x->image_buffer_R = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB);
    x->image_buffer_G = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB);
    x->image_buffer_B = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB);

    // Initialize the atom buffers for output
    x->atom_buffer_R = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB * sizeof(t_atom));
    x->atom_buffer_G = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB * sizeof(t_atom));
    x->atom_buffer_B = allocate_and_check((t_object*)x, CIS_MAX_PIXELS_NB * sizeof(t_atom));
    x->atom_IMU_Ax = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_IMU_Ay = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_IMU_Az = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_IMU_Gx = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_IMU_Gy = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_IMU_Gz = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_HID_B1 = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_HID_B2 = allocate_and_check((t_object*)x, sizeof(t_atom));
    x->atom_HID_B3 = allocate_and_check((t_object*)x, sizeof(t_atom));

    // Verify that all allocations were successful
    if (!x->image_buffer_R || !x->image_buffer_G || !x->image_buffer_B || !x->atom_buffer_R ||
        !x->atom_buffer_G || !x->atom_buffer_B || !x->atom_IMU_Ax || !x->atom_IMU_Ay || !x->atom_IMU_Az ||
        !x->atom_IMU_Gx || !x->atom_IMU_Gy || !x->atom_IMU_Gz || !x->atom_HID_B1 || !x->atom_HID_B2 || !x->atom_HID_B3)
    {
        cisReceive_cleanup(x);
        return NULL;
    }

    // Initialize the LED states to default values
    for (int i = 0; i < 3; i++)
    {
        x->leds[i].brightness_1 = 0;
        x->leds[i].time_1 = 0;
        x->leds[i].glide_1 = 0;
        x->leds[i].brightness_2 = 0;
        x->leds[i].time_2 = 0;
        x->leds[i].glide_2 = 0;
        x->leds[i].blink_count = 0;
    }

    // Initialize sockets and set up UDP reception
    if (syssock_set(x) < 0)
    {
        cisReceive_cleanup(x);
        return NULL;
    }

    // Create outlets
    x->outlet_sync = outlet_new(x, NULL);
    x->outlet_B = outlet_new(x, NULL);
    x->outlet_G = outlet_new(x, NULL);
    x->outlet_R = outlet_new(x, NULL);
    x->outlet_IMU = outlet_new(x, NULL);
    x->outlet_HID = outlet_new(x, NULL);
    
    return x;
}

void cisReceive_free(t_cisReceive *x)
{
    // Stop listening and close socket
    if (x->listener)
    {
        x->listening = false;
        if (x->fd)
        {
            syssock_dropmulticast(x->fd, x->multicast);
            syssock_close(x->fd);
            x->fd = 0; // Ensure the descriptor is reset
        }
        systhread_join(x->listener, NULL);
        x->listener = NULL;
    }

    // Free buffers and atom buffers
    cisReceive_cleanup(x);
}

void cisReceiveassist(t_cisReceive *x, void *b, long m, long a, char *s)
{
    if (m == ASSIST_INLET)
    {
        sprintf(s, "Input Inlet for Commands");
    }
    else
    {
        // ASSIST_OUTLET
        switch (a)
        {
            case 0:
                sprintf(s, "HID Data Output");
                break;
            case 1:
                sprintf(s, "IMU Data Output");
                break;
            case 2:
                sprintf(s, "Red Channel Output");
                break;
            case 3:
                sprintf(s, "Green Channel Output");
                break;
            case 4:
                sprintf(s, "Blue Channel Output");
                break;
            case 5:
                sprintf(s, "Synchronisation Bang");
                break;
            default:
                sprintf(s, "Unknown Outlet");
                break;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// MESSAGE LISTENER
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void cisReceive_readStartupInfo(t_cisReceive *x, void *data, uint16_t length)
{
    if (length == sizeof(struct packet_StartupInfo))
    {
        struct packet_StartupInfo *packet = (struct packet_StartupInfo *)data;

        // Store packet ID and version info in x structure
        x->startup_packet_id = packet->packet_id;
        strncpy(x->version_info, (const char *)packet->version_info, sizeof(x->version_info) - 1);
        x->version_info[sizeof(x->version_info) - 1] = '\0'; // Ensure it's a valid string

        // Post a message or perform other actions based on the startup info received
        post("Startup Packet ID: %u, Version Info: %s", packet->packet_id, x->version_info);
    }
    else
    {
        object_error((t_object*)x, "Invalid Startup Info packet length.");
    }
}

void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length)
{
    static uint32_t curr_line_id = 0;
    static uint32_t last_line_id = -1; // Track the last completed line ID
    static bool received_fragments[UDP_MAX_NB_PACKET_PER_LINE] = {0};
    static uint32_t fragment_count = 0; // Count the number of received fragments for the current line
    static uint32_t offset = 0;
    static bool bang_sent = false; // Track if bang has been sent for the current line

    // Ensure that the data length matches the expected packet size
    if (length != sizeof(struct packet_Image))
    {
        return;
    }

    struct packet_Image *packet = (struct packet_Image *)data;

    // Check if the line has already been processed completely
    if (last_line_id == packet->line_id)
    {
        return; // Skip processing this packet if it corresponds to a completed line
    }

    // Check if we are receiving a new line
    if (curr_line_id != packet->line_id)
    {
        curr_line_id = packet->line_id;
        memset(received_fragments, 0, packet->total_fragments * sizeof(bool));
        fragment_count = 0; // Reset the fragment counter for the new line
        bang_sent = false;  // Reset the bang flag for the new line
    }

    // Calculate the offset and copy the fragment data
    offset = packet->fragment_id * packet->fragment_size;

    // Only process the fragment if it hasn't already been received
    if (!received_fragments[packet->fragment_id])
    {
        received_fragments[packet->fragment_id] = TRUE;
        fragment_count++;

        memcpy(&x->image_buffer_R[offset], packet->imageData_R, packet->fragment_size);
        memcpy(&x->image_buffer_G[offset], packet->imageData_G, packet->fragment_size);
        memcpy(&x->image_buffer_B[offset], packet->imageData_B, packet->fragment_size);
    }

    // Check if all fragments have been received and bang hasn't been sent yet
    if (fragment_count == packet->total_fragments && !bang_sent)
    {
        // Double-check that all fragments are actually marked as received
        for (uint32_t i = 0; i < packet->total_fragments; i++)
        {
            if (!received_fragments[i])
            {
                return; // Exit if any fragment is missing
            }
        }

        // Mark this line as completed to avoid future re-processing
        last_line_id = packet->line_id;
        bang_sent = true; // Set the flag to prevent multiple bangs

        // Prepare the data for output
        for (int i = 0; i < CIS_MAX_PIXELS_NB; i++)
        {
            atom_setlong(&(x->atom_buffer_R[i]), (long)x->image_buffer_R[i]);
            atom_setlong(&(x->atom_buffer_G[i]), (long)x->image_buffer_G[i]);
            atom_setlong(&(x->atom_buffer_B[i]), (long)x->image_buffer_B[i]);
        }

        // Output the RGB data
        outlet_list(x->outlet_R, NULL, CIS_MAX_PIXELS_NB, x->atom_buffer_R);
        outlet_list(x->outlet_G, NULL, CIS_MAX_PIXELS_NB, x->atom_buffer_G);
        outlet_list(x->outlet_B, NULL, CIS_MAX_PIXELS_NB, x->atom_buffer_B);

        // Send a synchronization signal
        outlet_bang(x->outlet_sync);
    }
}

void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length)
{
    if (length == sizeof(struct packet_IMU))
    {
        struct packet_IMU *packet = (struct packet_IMU *)data;
        
        x->IMU_Ax = packet->acc[0];
        x->IMU_Ay = packet->acc[1];
        x->IMU_Az = packet->acc[2];
        
        x->IMU_Gx = packet->gyro[0];
        x->IMU_Gy = packet->gyro[1];
        x->IMU_Gz = packet->gyro[2];
        
        //post("ID: %d", packet->packet_id);
        
        //post("acc X: %f", packet->acc[0]);
        //post("acc Y: %f", packet->acc[1]);
        //post("acc Z: %f", packet->acc[2]);
        
        //post("gyro X: %f", packet->gyro[0]);
        //post("gyro Y: %f", packet->gyro[1]);
        //post("gyro Z: %f", packet->gyro[2]);
        
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
    }
}

void cisReceive_readButtonData(t_cisReceive *x, void *data, uint16_t length)
{
    if (length == sizeof(struct packet_Button))
    {
        struct packet_Button *packet = (struct packet_Button *)data;

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
        
        atom_setlong(x->atom_HID_B1, (long)x->HID_B1);
        atom_setlong(x->atom_HID_B2, (long)x->HID_B2);
        atom_setlong(x->atom_HID_B3, (long)x->HID_B3);
        send_data_to_outlet(x, s_HID_B1, 1, x->atom_HID_B1);
        send_data_to_outlet(x, s_HID_B2, 1, x->atom_HID_B2);
        send_data_to_outlet(x, s_HID_B3, 1, x->atom_HID_B3);
    }
    else
    {
        object_error((t_object*)x, "Invalid HID Data packet length.");
    }
}

void cisReceive_read(t_cisReceive *x)
{
    uint8_t msgbuf[sizeof(struct packet_Image)];
    uint32_t nbytes;
    socklen_t addrlen = sizeof(x->addr);

    while (x->listening)
    {
        nbytes = (uint32_t)recvfrom(x->fd, msgbuf, sizeof(struct packet_Image), 0, (struct sockaddr *) &x->addr, &addrlen);

        if (nbytes < 0)
        {
            object_error((t_object*)x, "Error receiving data: %d", errno);
            return;
        }

        switch (msgbuf[0])
        {
            case STARTUP_INFO_HEADER:
                if (nbytes >= sizeof(struct packet_StartupInfo))
                {
                    cisReceive_readStartupInfo(x, msgbuf, nbytes);
                }
                else
                {
                    object_error((t_object*)x, "Malformed STARTUP_INFO_HEADER packet.");
                }
                break;
            case IMAGE_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_Image))
                {
                    cisReceive_readImageData(x, msgbuf, nbytes);
                }
                else
                {
                    object_error((t_object*)x, "Malformed IMAGE_DATA_HEADER packet.");
                }
                break;
            case IMU_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_IMU))
                {
                    cisReceive_readImuData(x, msgbuf, nbytes);
                }
                else
                {
                    object_error((t_object*)x, "Malformed IMU_DATA_HEADER packet.");
                }
                break;
            case BUTTON_DATA_HEADER:
                if (nbytes >= sizeof(struct packet_Button))
                {
                    cisReceive_readButtonData(x, msgbuf, nbytes);
                }
                else
                {
                    object_error((t_object*)x, "Malformed HID_DATA_HEADER packet.");
                }
                break;
            default:
                object_error((t_object*)x, "Unknown packet header.");
                return;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// SEND DATA TO OUTLET
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void send_data_to_outlet(void* object, t_symbol* sel, int argc, t_atom* argv)
{
    t_cisReceive *x = (t_cisReceive *)object;

    if (sel == s_Line_R)
    {
        outlet_anything(x->outlet_R, s_Line_R, argc, argv);
    }
    else if (sel == s_Line_G)
    {
        outlet_anything(x->outlet_G, s_Line_G, argc, argv);
    }
    else if (sel == s_Line_B)
    {
        outlet_anything(x->outlet_B, s_Line_B, argc, argv);
    }
    else if (sel == s_IMU_Ax)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Ax, argc, argv);
    }
    else if (sel == s_IMU_Ay)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Ay, argc, argv);
    }
    else if (sel == s_IMU_Az)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Az, argc, argv);
    }
    else if (sel == s_IMU_Gx)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Gx, argc, argv);
    }
    else if (sel == s_IMU_Gy)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Gy, argc, argv);
    }
    else if (sel == s_IMU_Gz)
    {
        outlet_anything(x->outlet_IMU, s_IMU_Gz, argc, argv);
    }
    else if (sel == s_HID_B1)
    {
        outlet_anything(x->outlet_HID, s_HID_B1, argc, argv);
    }
    else if (sel == s_HID_B2)
    {
        outlet_anything(x->outlet_HID, s_HID_B2, argc, argv);
    }
    else if (sel == s_HID_B3)
    {
        outlet_anything(x->outlet_HID, s_HID_B3, argc, argv);
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// SYSSOCK EXTENSIONS
//---------------------------------------------------------------------------------------------------------------------------------------------------------

int syssock_set(t_cisReceive *x)
{
    if (syssock_init())
    {
        object_error((t_object*)x, "Cannot start socket API");
        return -1;
    }

    x->fd = syssock_socket(AF_INET, SOCK_DGRAM, 0);
    if (x->fd < 0)
    {
        object_error((t_object*)x, "Cannot start UDP socket");
        return -1;
    }

    if (syssock_reuseaddr(x->fd))
    {
        object_error((t_object*)x, "Cannot set socket REUSEADDR option");
        return -1;
    }

    memset(&x->addr, 0, sizeof(x->addr));
    x->addr.sin_family = AF_INET;
    x->addr.sin_addr.s_addr = (in_addr_t)syssock_htonl(INADDR_ANY);
    x->addr.sin_port = htons(x->port);

    if (syssock_bind(x->fd, &x->addr))
    {
        object_error((t_object*)x, "Cannot bind to port %d", x->port);
        return -1;
    }
    else
    {
        post("Socket bound to port %d", x->port); // Confirmation of binding to port
    }

    x->listening = true;
    systhread_create((method)cisReceive_read, x, 0, 0, 0, &(x->listener));

    return 0;
}

int syssock_addmulticast(t_syssocket sockfd, char* ip)
{
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = syssock_inet_addr(ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    return syssock_setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
}

int syssock_dropmulticast(t_syssocket sockfd, char* ip)
{
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = syssock_inet_addr(ip);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    return syssock_setsockopt(sockfd, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// TCP Leds handle
//---------------------------------------------------------------------------------------------------------------------------------------------------------

int set_socket_nonblocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1)
    {
        return -1;
    }
    return fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

int set_socket_blocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1)
    {
        return -1;
    }
    return fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK);
}

int tcp_connect_nonblocking(const char *server_ip, int server_port, int timeout_seconds)
{
    int sockfd;
    struct sockaddr_in server_addr;

    // Create a TCP socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("Socket creation error");
        return -1;
    }

    // Set socket to non-blocking mode
    set_socket_nonblocking(sockfd);

    // Configure server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0)
    {
        perror("Invalid address or unsupported");
        close(sockfd);
        return -1;
    }

    // Initiate connection
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
        if (errno != EINPROGRESS)
        {
            perror("Connection error");
            close(sockfd);
            return -1;
        }
    }

    // Wait for connection with select
    fd_set writefds;
    struct timeval timeout;
    FD_ZERO(&writefds);
    FD_SET(sockfd, &writefds);
    timeout.tv_sec = timeout_seconds;
    timeout.tv_usec = 0;

    int ret = select(sockfd + 1, NULL, &writefds, NULL, &timeout);
    if (ret <= 0)
    {
        perror("Connection timeout or error");
        close(sockfd);
        return -1;
    }

    // Connection successful, set socket back to blocking mode
    set_socket_blocking(sockfd);

    return sockfd;
}

ssize_t tcp_send(int sockfd, const void *data, size_t length)
{
    ssize_t sent_bytes = send(sockfd, data, length, 0);
    if (sent_bytes < 0)
    {
        if (errno == EWOULDBLOCK || errno == EAGAIN)
        {
            fprintf(stderr, "Error: server not responding (timeout reached)\n");
        }
        else
        {
            perror("Data send error");
        }
    }
    return sent_bytes;
}

void *tcp_send_thread(void *args)
{
    struct packet_Leds *packet = (struct packet_Leds *)args;
    int sockfd = tcp_connect_nonblocking("192.168.0.10", 5000, 5);
    if (sockfd >= 0)
    {
        if (tcp_send(sockfd, packet, sizeof(struct packet_Leds)) < 0)
        {
            fprintf(stderr, "Failed to send LED packet\n");
        }
        close(sockfd);
    }
    else
    {
        fprintf(stderr, "Unable to connect to server\n");
    }

    free(packet); // Free allocated memory for the packet after sending
    return NULL;
}

void start_tcp_send_thread(ledIdTypeDef led_id, struct led_State led_state)
{
    t_systhread thread;
    static uint32_t packet_id = 0;

    // Allocate memory for the LED packet
    struct packet_Leds *packet = malloc(sizeof(struct packet_Leds));
    if (packet == NULL)
    {
        fprintf(stderr, "Memory allocation error for LED packet\n");
        return;
    }

    // Configure packet data
    packet->type = LED_DATA_HEADER; // Using the type for LED data
    packet->packet_id = packet_id++; // Increment packet ID
    packet->led_id = led_id;
    packet->led_state = led_state;

    // Create and detach the thread using systhread_create
    if (systhread_create((method)tcp_send_thread, packet, 0, 0, 0, &thread) == MAX_ERR_NONE)
    {
        systhread_detach(thread); // Detach the thread to avoid waiting for its termination
    }
    else
    {
        fprintf(stderr, "Thread creation error\n");
        free(packet); // Free memory if thread creation fails
    }
}

void cisReceive_ledCommand(t_cisReceive *x, t_symbol *s, long argc, t_atom *argv)
{
    if (argc > 7)
    {
        object_error((t_object*)x, "Invalid number of arguments, expected 7 (brightness_1, time_1, glide_1, brightness_2, time_2, glide_2, blink_count)");
        return;
    }
    
    // Map the symbol to LED ID
    ledIdTypeDef led_id;
    if (s == gensym("LED_1"))
    {
        led_id = LED_1;
    } else if (s == gensym("LED_2"))
    {
        led_id = LED_2;
    } else if (s == gensym("LED_3"))
    {
        led_id = LED_3;
    } else
    {
        object_error((t_object*)x, "Unknown LED ID symbol");
        return;
    }
    
    struct led_State led_state;
    led_state.brightness_1 = (uint16_t)atom_getlong(argv);
    led_state.time_1 = (uint16_t)atom_getlong(argv + 1);
    led_state.glide_1 = (uint16_t)atom_getlong(argv + 2);
    led_state.brightness_2 = (uint16_t)atom_getlong(argv + 3);
    led_state.time_2 = (uint16_t)atom_getlong(argv + 4);
    led_state.glide_2 = (uint16_t)atom_getlong(argv + 5);
    led_state.blink_count = (uint32_t)atom_getlong(argv + 6);
    
    start_tcp_send_thread(led_id, led_state);
}

t_max_err led_attr_get(t_cisReceive *x, void *attr, long *ac, t_atom **av)
{
    if (!ac || !av)
        return MAX_ERR_GENERIC;

    *ac = 1;
    if (!(*av = (t_atom *)sysmem_newptr(sizeof(t_atom))))
        return MAX_ERR_GENERIC;

    t_symbol *attr_name_symbol = (t_symbol *)object_method((t_object *)attr, gensym("getname"));
    if (attr_name_symbol)
    {
        const char *attr_name = attr_name_symbol->s_name;
        ledIdTypeDef led;
        const char *param_name;

        // Determine which LED and parameter
        if (strncmp(attr_name, "led1_", 5) == 0)
        {
            led = LED_1;
            param_name = attr_name + 5;
        }
        else if (strncmp(attr_name, "led2_", 5) == 0)
        {
            led = LED_2;
            param_name = attr_name + 5;
        }
        else if (strncmp(attr_name, "led3_", 5) == 0)
        {
            led = LED_3;
            param_name = attr_name + 5;
        }
        else
        {
            object_error((t_object*)x, "Unrecognized LED in getter.");
            return MAX_ERR_GENERIC;
        }

        uint32_t value = 0;
        if (strcmp(param_name, "brightness_1") == 0)
        {
            value = x->leds[led].brightness_1;
        }
        else if (strcmp(param_name, "time_1") == 0)
        {
            value = x->leds[led].time_1;
        }
        else if (strcmp(param_name, "glide_1") == 0)
        {
            value = x->leds[led].glide_1;
        }
        else if (strcmp(param_name, "brightness_2") == 0)
        {
            value = x->leds[led].brightness_2;
        }
        else if (strcmp(param_name, "time_2") == 0)
        {
            value = x->leds[led].time_2;
        }
        else if (strcmp(param_name, "glide_2") == 0)
        {
            value = x->leds[led].glide_2;
        }
        else if (strcmp(param_name, "blink_count") == 0)
        {
            value = x->leds[led].blink_count;
        }
        else
        {
            object_error((t_object*)x, "Unrecognized attribute in getter.");
            return MAX_ERR_GENERIC;
        }

        atom_setlong(*av, (t_atom_long)value);
    }
    else
    {
        object_error((t_object*)x, "Unable to retrieve attribute name in getter.");
        return MAX_ERR_GENERIC;
    }

    return MAX_ERR_NONE;
}

t_max_err led_attr_set(t_cisReceive *x, void *attr, long ac, t_atom *av)
{
    if (ac && av)
    {
        long value = atom_getlong(av);
        ledIdTypeDef led;
        const char *param_name;

        t_symbol *attr_name_symbol = (t_symbol *)object_method((t_object *)attr, gensym("getname"));
        if (attr_name_symbol)
        {
            const char *attr_name = attr_name_symbol->s_name;

            // Determine which LED and parameter
            if (strncmp(attr_name, "led1_", 5) == 0)
            {
                led = LED_1;
                param_name = attr_name + 5;
            }
            else if (strncmp(attr_name, "led2_", 5) == 0)
            {
                led = LED_2;
                param_name = attr_name + 5;
            }
            else if (strncmp(attr_name, "led3_", 5) == 0)
            {
                led = LED_3;
                param_name = attr_name + 5;
            }
            else
            {
                object_error((t_object*)x, "Unrecognized LED in setter.");
                return MAX_ERR_GENERIC;
            }

            if (strcmp(param_name, "brightness_1") == 0)
            {
                x->leds[led].brightness_1 = (uint16_t)value;
            }
            else if (strcmp(param_name, "time_1") == 0)
            {
                x->leds[led].time_1 = (uint16_t)value;
            }
            else if (strcmp(param_name, "glide_1") == 0)
            {
                x->leds[led].glide_1 = (uint16_t)value;
            }
            else if (strcmp(param_name, "brightness_2") == 0)
            {
                x->leds[led].brightness_2 = (uint16_t)value;
            }
            else if (strcmp(param_name, "time_2") == 0)
            {
                x->leds[led].time_2 = (uint16_t)value;
            }
            else if (strcmp(param_name, "glide_2") == 0)
            {
                x->leds[led].glide_2 = (uint16_t)value;
            }
            else if (strcmp(param_name, "blink_count") == 0)
            {
                x->leds[led].blink_count = (uint32_t)value;
            }
            else
            {
                object_error((t_object*)x, "Unrecognized attribute in setter.");
                return MAX_ERR_GENERIC;
            }

            // Send the updated LED state
            start_tcp_send_thread(led, x->leds[led]);
        }
        else
        {
            object_error((t_object*)x, "Unable to retrieve attribute name in setter.");
            return MAX_ERR_GENERIC;
        }
    }
    return MAX_ERR_NONE;
}


