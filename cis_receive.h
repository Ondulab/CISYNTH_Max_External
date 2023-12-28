#ifndef cis_receive_h
#define cis_receive_h

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES (see http://www.zachburlingame.com/2011/05/resolving-redefinition-errors-betwen-ws2def-h-and-winsock-h/)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef WIN_VERSION
#define MAXAPI_USE_MSCRT
#endif

#ifndef CIS_RECEIVE
#define CIS_RECEIVE
#endif

#include "ext_syssock.h"	    // CNMAT crossplatform socket library - should be first to avoid winsock2 redeclaration problem
#include "jit.common.h"

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#define UDP_HEADER_SIZE                          (1)//uint32
#define UDP_NB_PACKET_PER_LINE                   (6)
#define UDP_NB_BYTES_PER_PIXELS                  (4)
#define UDP_PACKET_SIZE                          (((((CIS_PIXELS_NB) / UDP_NB_PACKET_PER_LINE)) + (UDP_HEADER_SIZE)) * (UDP_NB_BYTES_PER_PIXELS))

#define PORT                                     (55151)    //The port on which to listen for incoming data

#ifdef CIS_400DPI
#define CIS_PIXELS_PER_LINE                      (1152)
#else
#define CIS_PIXELS_PER_LINE                      (576)
#endif

#define CIS_ADC_OUT_LINES                        (3)

#define CIS_PIXELS_NB                            ((CIS_PIXELS_PER_LINE * CIS_ADC_OUT_LINES))

#define DEFAULT_MULTI	"192.168.0.1"
#define DEFAULT_PORT	PORT

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTURE
//---------------------------------------------------------------------------------------------------------------------------------------------------------

typedef struct _cisReceive {    // defines our object's internal variables for each instance in a patch
	t_object    s_ob;			// object header - ALL objects MUST begin with this...
	t_syssocket fd;				// UDP socket
	t_sysaddr   addr;			// Address structure
	int listening;				// listening state
	t_systhread listener;		// thread
	char* multicast;			// multicast address
	int port;					// UDP reveiving port
    void* outlet_list;          // Outlet pour la liste d'atoms
    void* outlet_jit;           // Outlet pour la matrice Jitter
    uint8_t* image_buffer;      // Buffer pour stocker une ligne complète de l'image
    t_jit_object *matrix;
} t_cisReceive;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// PROTOTYPES
//---------------------------------------------------------------------------------------------------------------------------------------------------------

void *cisReceive_new(t_symbol*, long, t_atom*);
void cisReceive_free(t_cisReceive*);
void cisReceive_assist(t_cisReceive*, void*, long, long, char*);
void cisReceive_read(t_cisReceive*);

int syssock_set(t_cisReceive*);
int syssock_addmulticast(t_syssocket, char*);
int syssock_dropmulticast(t_syssocket, char*);

#endif /* cis_send_h */
