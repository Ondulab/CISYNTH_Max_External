/*
	udpReceive.c - receive data from Ondulab CIS devices

	this object has one inlet and one outlet
	it decodes formatted image lines from CIS in udp
	it send to MAX outlet ARGB array

	https://github.com/CNMAT/CNMAT-Externs source code is a great source of inspiration
    https://github.com/siteswapjuggler/smartball-externals source code of inspiration
    https://github.com/Ondulab/SSS_CIS source code of CIS device
*/

#include "cis_receive.h"

void cisReceive_readStartupInfo(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length);
void cisReceive_readHidData(t_cisReceive *x, void *data, uint16_t length);

//---------------------------------------------------------------------------------------------------------------------------------------------------------
//  INSTANCE DECLARATION
//---------------------------------------------------------------------------------------------------------------------------------------------------------

t_class *cisReceiveclass;		// global pointer to the object class - so max can reference the object

void ext_main(void *r)
{
	t_class *c;																											// pointer to a class type
	c = class_new("cis_receive", (method)cisReceive_new, (method)cisReceive_free, sizeof(t_cisReceive), 0L, A_GIMME, 0); 	// class_new() loads our external's class into Max's memory so it can be used in a patch
	class_register(CLASS_BOX, c);																						// register to CLASS_BOX type for max environment
	cisReceiveclass = c;
	post("cis_receive v0.06 - 31.12.2023");
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
        post("Defaut port :%d", DEFAULT_PORT);
	}
	else {
	ARG_FAULT:
		object_error((t_object*)x, "Cannot start with those arguments ");
		goto NOT_VALID;
	}
    
    // Initialiser le buffer d'image
    x->image_buffer_R = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    x->image_buffer_G = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    x->image_buffer_B = (uint8_t *)sysmem_newptr(CIS_PIXELS_NB);
    
    if (!x->image_buffer_R) {
        object_error((t_object*)x, "Erreur d'allocation pour image_buffer");
        return NULL;
    }
    
    if (!x->image_buffer_G) {
        object_error((t_object*)x, "Erreur d'allocation pour image_buffer");
        return NULL;
    }
    
    if (!x->image_buffer_B) {
        object_error((t_object*)x, "Erreur d'allocation pour image_buffer");
        return NULL;
    }

	// HANDLE SOCKET INITIATION
	if (syssock_set(x) < 0) goto NOT_VALID;
	
	// CREATE OUTLETS
    x->outlet_R = outlet_new(x, NULL);
    x->outlet_G = outlet_new(x, NULL);
    x->outlet_B = outlet_new(x, NULL);
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
		}
		systhread_join(x->listener, NULL);
		x->listener = NULL;
	}
    // Libérer le buffer d'image
    if (x->image_buffer_R) {
        sysmem_freeptr(x->image_buffer_R);
    }
    if (x->image_buffer_G) {
        sysmem_freeptr(x->image_buffer_G);
    }
    if (x->image_buffer_B) {
        sysmem_freeptr(x->image_buffer_B);
    }
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
    
}

void cisReceive_readImageData(t_cisReceive *x, void *data, uint16_t length) {
    static uint32_t curr_line_id = 0;
    static bool received_fragments_R[UDP_NB_PACKET_PER_LINE] = {0}; //todo rendre intelligent en récuperant
    static bool received_fragments_G[UDP_NB_PACKET_PER_LINE] = {0};
    static bool received_fragments_B[UDP_NB_PACKET_PER_LINE] = {0};
    static uint32_t offset = 0;
    static uint8_t color = 0;
    
    if (length == sizeof(struct packet_Image)) {
        struct packet_Image *packet = (struct packet_Image *)data;

        if (curr_line_id != packet->line_id)
        {
            curr_line_id = packet->line_id;
            memset(received_fragments_R, 0, packet->total_fragments * sizeof(bool));
            memset(received_fragments_G, 0, packet->total_fragments * sizeof(bool));
            memset(received_fragments_B, 0, packet->total_fragments * sizeof(bool));
        }
        
        offset = packet->fragment_id * packet->fragment_size;
        color = packet->imageColor;
        
        switch (color) {
            case IMAGE_COLOR_R:
                for (uint32_t i = 0; i < packet->fragment_size; i++) {
                    x->image_buffer_R[offset + i] = packet->imageData[i];
                }
                received_fragments_R[packet->fragment_id] = TRUE;
                break;
            case IMAGE_COLOR_G:
                for (uint32_t i = 0; i < packet->fragment_size; i++) {
                    x->image_buffer_G[offset + i] = packet->imageData[i];
                }
                received_fragments_G[packet->fragment_id] = TRUE;
                break;
            case IMAGE_COLOR_B:
                for (uint32_t i = 0; i < packet->fragment_size; i++) {
                    x->image_buffer_B[offset + i] = packet->imageData[i];
                }
                received_fragments_B[packet->fragment_id] = TRUE;
                break;
            default:
                // error managment
                return;
        }
        
        x->line_complete = TRUE; // Commencez par supposer que la ligne est complète

        for (int i = 0; i < packet->total_fragments; i++) {
            if (!received_fragments_R[i] || !received_fragments_G[i] || !received_fragments_B[i]) {
                x->line_complete = FALSE; // Dès qu'un fragment manquant est trouvé, marquez la ligne comme incomplète
                break; // Pas besoin de vérifier les autres fragments
            }
        }
    }
}

void cisReceive_readImuData(t_cisReceive *x, void *data, uint16_t length) {
    
    if (length == sizeof(struct packet_IMU)) {
        struct packet_IMU *packet = (struct packet_IMU *)data;
        
        //post("ID: %d", packet->packet_id);
        
        //post("acc X: %d", packet->acc[0]);
        //post("acc Y: %d", packet->acc[1]);
        //post("acc Z: %d", packet->acc[2]);
        
        //post("gyro X: %d", packet->gyro[0]);
        //post("gyro Y: %d", packet->gyro[1]);
        //post("gyro Z: %d", packet->gyro[2]);
    }
}

void cisReceive_readHidData(t_cisReceive *x, void *data, uint16_t length) {
    
}
    
void cisReceive_read(t_cisReceive *x) {
#ifndef RGBA_BUFFER
    uint8_t msgbuf[UDP_PACKET_SIZE];
    uint32_t nbytes;
    socklen_t addrlen = sizeof(x->addr);

    while (x->listening) {
        nbytes = (uint32_t)recvfrom(x->fd, msgbuf, UDP_PACKET_SIZE, 0, (struct sockaddr *) &x->addr, &addrlen);
        //nbytes = (uint32_t)recvfrom(x->fd, msgbuf, sizeof(struct packet_Image), 0, (struct sockaddr *) &x->addr, &addrlen);

        switch (msgbuf[0]) {
            case STARTUP_INFO_HEADER:
                cisReceive_readStartupInfo(x, msgbuf, nbytes);
                break;
            case IMAGE_DATA_HEADER:
                cisReceive_readImageData(x, msgbuf, nbytes);
                break;
            case IMU_DATA_HEADER:
                cisReceive_readImuData(x, msgbuf, nbytes);
                break;
            case HID_DATA_HEADER:
                cisReceive_readHidData(x, msgbuf, nbytes);
                break;
            default:
                // error managment
                return;
        }
        
        if (x->line_complete == TRUE) { //une ligne complète est reçue
            //post("start line sending"); // Afficher le nombre de pixels
            x->line_complete = FALSE;
            
            // UDP_NB_PACKET_PER_LINE * UDP_PACKET_SIZE donne la taille totale du buffer en octets
            static int size = CIS_PIXELS_NB;
            
            // Créer un tableau de t_atom de la taille appropriée
            t_atom *atom_buffer_R = (t_atom *)sysmem_newptr(size * sizeof(t_atom));
            t_atom *atom_buffer_G = (t_atom *)sysmem_newptr(size * sizeof(t_atom));
            t_atom *atom_buffer_B = (t_atom *)sysmem_newptr(size * sizeof(t_atom));
            
            // Remplir le tableau de t_atom avec les données de votre buffer
            for(int i = 0; i < size; i++) {
                atom_setlong(atom_buffer_R + i, (long)x->image_buffer_R[i]);
                atom_setlong(atom_buffer_G + i, (long)x->image_buffer_G[i]);
                atom_setlong(atom_buffer_B + i, (long)x->image_buffer_B[i]);
            }
            
            // Envoyer la liste d'atoms
            outlet_list(x->outlet_R, NULL, size, atom_buffer_R);
            outlet_list(x->outlet_G, NULL, size, atom_buffer_G);
            outlet_list(x->outlet_B, NULL, size, atom_buffer_B);
            
            // Libérer la mémoire allouée pour le tableau de t_atom
            sysmem_freeptr(atom_buffer_R);
            sysmem_freeptr(atom_buffer_G);
            sysmem_freeptr(atom_buffer_B);
        }
    }
        
#else
    uint8_t msgbuf[UDP_PACKET_SIZE];
    uint32_t nbytes;
    socklen_t addrlen = sizeof(x->addr);

    while (x->listening) {
        nbytes = (uint32_t)recvfrom(x->fd, msgbuf, UDP_PACKET_SIZE, 0, (struct sockaddr *) &x->addr, &addrlen);
        uint32_t header = *((uint32_t *)msgbuf);
    
        if (nbytes == UDP_PACKET_SIZE) {
            //post("cisReceive data length: %d", nbytes); // Afficher la longueur des données
            
            // Extraction de l'en-tête
            uint32_t header = *((uint32_t *)msgbuf);
            //post("header: %d", header); // Afficher le numéro de packet

            // Calculer le nombre de pixels à traiter
            uint32_t numPixels = (nbytes - sizeof(int32_t)) / sizeof(int32_t);
            //post("pixels number: %d", numPixels); // Afficher le nombre de pixels

            // Traitement des données (conversion de RGBA à ARGB)
            for (uint32_t i = 0; i < numPixels; i++) {
                // Extraction du pixel en format RGBA
                uint32_t rgba_pixel = *(uint32_t *)(msgbuf + ((UDP_HEADER_SIZE * sizeof(int32_t)) + (i * sizeof(uint32_t))));
                //post("rgba_pixel:%08X",rgba_pixel);

                // Extraction des composants individuels
                uint8_t alpha = (rgba_pixel >> 24) & 0xFF; // Extract red
                uint8_t blue = (rgba_pixel >> 16) & 0xFF; // Extract green
                uint8_t green = (rgba_pixel >> 8) & 0xFF; // Extract blue
                uint8_t red = rgba_pixel & 0xFF; // Extract alpha
                
                // Enregistrer les valeurs de canal dans le buffer d'image
                x->image_buffer_R[header + i] = red;
                x->image_buffer_G[header + i] = green;
                x->image_buffer_B[header + i] = blue;
                
                //post("Pixel:%d A:%d R:%d G:%d B:%d", i, alpha, red, green, blue);
            }
            
            if (header == (CIS_PIXELS_NB - (CIS_PIXELS_NB / UDP_NB_PACKET_PER_LINE))) { // header à 1440, une ligne complète est reçue
                post("start line sending"); // Afficher le nombre de pixels
                
                // UDP_NB_PACKET_PER_LINE * UDP_PACKET_SIZE donne la taille totale du buffer en octets
                int size = CIS_PIXELS_NB;

                // Créer un tableau de t_atom de la taille appropriée
                t_atom *atom_buffer_R = (t_atom *)sysmem_newptr(size * sizeof(t_atom));
                t_atom *atom_buffer_G = (t_atom *)sysmem_newptr(size * sizeof(t_atom));
                t_atom *atom_buffer_B = (t_atom *)sysmem_newptr(size * sizeof(t_atom));

                // Remplir le tableau de t_atom avec les données de votre buffer
                for(int i = 0; i < size; i++) {
                    atom_setlong(atom_buffer_R + i, (long)x->image_buffer_R[i]);
                    atom_setlong(atom_buffer_G + i, (long)x->image_buffer_G[i]);
                    atom_setlong(atom_buffer_B + i, (long)x->image_buffer_B[i]);
                }

                // Envoyer la liste d'atoms
                outlet_list(x->outlet_R, NULL, size, atom_buffer_R);
                outlet_list(x->outlet_G, NULL, size, atom_buffer_G);
                outlet_list(x->outlet_B, NULL, size, atom_buffer_B);

                // Libérer la mémoire allouée pour le tableau de t_atom
                sysmem_freeptr(atom_buffer_R);
                sysmem_freeptr(atom_buffer_G);
                sysmem_freeptr(atom_buffer_B);

#if(0)
                for (uint32_t i = 0; i < CIS_PIXELS_NB; i++) {
                    post("Pixel:%d A:%d R:%d G:%d B:%d", i,
                                                            x->image_buffer[i * sizeof(int32_t)],
                                                            x->image_buffer[(i * sizeof(int32_t)) + 1],
                                                            x->image_buffer[(i * sizeof(int32_t)) + 2],
                                                            x->image_buffer[(i * sizeof(int32_t)) + 3]);
                }
#endif
            }
        } else {
            post("Data length is too short, ignoring");
        }
    }
#endif
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
        post("Socket bound to port %d", x->port); // Confirmation de la liaison au port
    }

    x->listening = true;
    systhread_create((method)cisReceive_read, x, 0, 8192, 0, &(x->listener));

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
