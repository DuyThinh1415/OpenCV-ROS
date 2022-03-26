#ifdef MAIN_PROCESS_H

#define MAIN_PROCESS_H

    //Define below use for debug prupose, only yes or no.
    #define debug_change_lane 0
    #define debug_process_image 1

    // ========================== Code part started ========================
    #define pixel(f,i,c) (int)(*f.ptr(i,c))

#endif