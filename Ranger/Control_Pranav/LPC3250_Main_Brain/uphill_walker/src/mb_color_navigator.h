#define true 1
#define false 0

/*
 	Track State Machine
		Barton track reigions
		

		Assumed starting point
				 || 
				 ||
			  	 \/
				    ~North
			   	 |	   I	    |
			   / |--------------| \ 
statler		 /	 				    \			 lynah
		   /						  \
	~West |					 	       |	 ~East
	  IV  |						 	   |	   II
		   \				  	      /
			 \	 				    /
			   \ |--------------| / 
			   	 |	   III	    |
					  ~South
						
		
*/								

enum location {north = 450, south = 452, east = 451, west = 453};
