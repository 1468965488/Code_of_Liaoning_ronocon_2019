#include "shoubing.h"
     


HB shoubing_init(int fun[4])
{ HB shoubing;
	u8 mode=0;
	//u8 mode0=0;
//	u8 mode1=0;
	if(fun[3]==0x0c)
		mode=1;   //Ò¡¸Ë
	else 
		 mode=2;  //°´¼ü
  if(mode==1)
  	{ shoubing.x=fun[2];
	    shoubing.y=fun[1];
			shoubing.mode=1;
	      }
	else{shoubing.x=fun[2];
	     shoubing.y=fun[1];
			 shoubing.mode=2;
		       }

     return shoubing;
 }
