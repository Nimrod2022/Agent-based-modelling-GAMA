/***
* Name: vierkaser
* Author: Nimrod Kibet
* Description: The project was developed under the instruction and guidance of Dr. Wallentin Gudrun, head of Spatial Smiluation research group at Z_GIS.
* Aim: The project goal was to introduce spatial simulation using agent-based modeling concepts.
***/

model vierkaser

/* model definitions */

global torus: false {
	//Flocking dynamic variables
	
	int number_of_cows <- 10 min: 3 max: 60; 	
	float min_separation <- 3.0  min: 0.1  max: 10.0 ;
	int max_separate_turn <- 5 min: 0 max: 20;
	int max_cohere_turn <- 5 min: 0 max: 20;
	int max_align_turn <- 8 min: 0 max: 20;
	float vision <- 30.0  min: 0.0  max: 70.0 ;	//end of added variables
	
	file pasture_file <- file("../includes/pasture.geojson");

	geometry shape <- envelope(pasture_file);
	
	geometry pasture_polygon;
	
	list<grass> pasture_cells;
	
	list<grass> grazed_list;
	
	//the step influences step length, but not speed
	float cow_speed <- 20.0 ;
	float cow_amplitude <- 60.0;
	float half_cow_speed <-10.0;
	
	string scenario ;
			
	//create the agents and objects with their variables
	init {		
		pasture_polygon <- geometry(pasture_file);
		
	
		create cows number:number_of_cows {
			location <- any_location_in(pasture_polygon);
			
	}
		
	//Create new species
		
		pasture_cells <- grass inside(pasture_polygon);
	}
	// Reflex to update biomass growth and dynamic color change
	reflex update_biomass {
		ask pasture_cells {
			if biomass < 10.0 {
				biomass <- biomass + 0.01;	
			}			
			color <- rgb([0, biomass * 15, 0]);	
		}
	}
}

species cows skills: [moving] {
	
	int age;
	geometry action_area;
	//cow visualization
	file cow_visualization <- file("../includes/cow.obj");
	//the cell that the cow grazes on at a particular time step
	grass grazed_grass ;
	//energy is gained by feeding grass, and lost by metabolism every time step
	float energy <- 4.0;
	
	
	// creating flocking variables
	list<cows> flockmates;
	cows nearest_neighbour;
	int avg_head;
	int avg_twds_mates ; //end of flocking variables

		
	//flocking reflex
	
	action flock {
    		
    		action_area <- circle (cow_speed) intersection cone(-30, 30);
    		// in case all flocking parameters are zero wander randomly 
    		    		 	
			if (max_separate_turn = 0 and max_cohere_turn = 0 and max_align_turn = 0 ) {
				do wander amplitude: 120 speed:cow_speed bounds:pasture_polygon;
			}
			// otherwise compute the heading for the next timestep in accordance to my flockmates
			else {
				// search for flockmates
				do find_flockmates ;
				// turn my heading to flock, if there are other agents in vision 
				if (not empty (flockmates)) {
					do find_nearest_neighbour;
					if (distance_to (self, nearest_neighbour) < min_separation) {
						do separate;
					}
					else {
						do align;
						do cohere;
					}
					// move forward in the new direction
					do move speed:cow_speed bounds:pasture_polygon;
				}
				// wander randomly, if there are no other agents in vision
				else {
					do wander amplitude: 120.0 speed:cow_speed bounds:pasture_polygon;
				}
			}			
	    } //end of flocking reflex
	    
	    // actions
	    //flockmates are defined spatially, within a buffer of vision
		action find_flockmates {
	        flockmates <- ((cows overlapping (circle(vision))) - self);
		}//end of find flockmates
		
		//find nearest neighbour
		action find_nearest_neighbour {
	        nearest_neighbour <- flockmates with_min_of(distance_to (self.location, each.location)); 
		}		
		
	    // separate from the nearest neighbour of flockmates
	    action separate  {
	    	do turn_away (nearest_neighbour towards self, max_separate_turn);
	    }
	
	    //Reflex to align the boid with the other boids in the range
	    action align  {
	    	avg_head <- avg_mate_heading () ;
	        do turn_towards (avg_head, max_align_turn);
	    }
	
	    //Reflex to apply the cohesion of the boids group in the range of the agent
	    action cohere  {
			avg_twds_mates <- avg_heading_towards_mates ();
			do turn_towards (avg_twds_mates, max_cohere_turn); 
	    }
		
		//end of actions
		
		 int avg_mate_heading {
    		list<cows> flockmates_insideShape <- flockmates where (each.destination != nil);
    		float x_component <- sum (flockmates_insideShape collect (each.destination.x - each.location.x));
    		float y_component <- sum (flockmates_insideShape collect (each.destination.y - each.location.y));
    		//if the flockmates vector is null, return my own, current heading
    		if (x_component = 0 and y_component = 0) {
    			return heading;
    		}
    		//else compute average heading of vector  		
    		else {
    			// note: 0-heading direction in GAMA is east instead of north! -> thus +90
    			return int(-1 * atan2 (x_component, y_component) + 90);
    		}	
	    }  
				
		  //compute the mean direction from me towards flockmates	    
	    int avg_heading_towards_mates {
	    	float x_component <- mean (flockmates collect (cos (towards(self.location, each.location))));
	    	float y_component <- mean (flockmates collect (sin (towards(self.location, each.location))));
	    	//if the flockmates vector is null, return my own, current heading
	    	if (x_component = 0 and y_component = 0) {
	    		return heading;
	    	}
    		//else compute average direction towards flockmates
	    	else {
	    		// note: 0-heading direction in GAMA is east instead of north! -> thus +90
	    		return int(-1 * atan2 (x_component, y_component) + 90);	
	    	}
	    } 	    
	    
	    // cohere
	    action turn_towards (int new_heading, int max_turn) {
			int subtract_headings <- new_heading - heading;
			if (subtract_headings < -180) {subtract_headings <- subtract_headings + 360;}
			if (subtract_headings > 180) {subtract_headings <- subtract_headings - 360;}
	    	do turn_at_most ((subtract_headings), max_turn);
	    }
	    
	    // separate
	    action turn_away (int new_heading, int max_turn) {
			int subtract_headings <- heading - new_heading;
			if (subtract_headings < -180) {subtract_headings <- subtract_headings + 360;}
			if (subtract_headings > 180) {subtract_headings <- subtract_headings - 360;}
	    	do turn_at_most ((-1 * subtract_headings), max_turn);
	    }
	    
	    // align
	    action turn_at_most (int turn, int max_turn) {
	    	if abs (turn) > max_turn {
	    		if turn > 0 {
	    			//right turn
	    			heading <- heading + max_turn;
	    		}
	    		else {
	    			//left turn
	    			heading <- heading - max_turn;
	    		}
	    	}
	    	else {
	    		heading <- heading + turn;
	    	} 
	    }	    
		
		// alternative arrow
		aspect arrow2 {
			if (destination != nil) {
				draw line([location, destination]) end_arrow: 2 color: color;
			}
		}
		
		// additional vision buffer 
		aspect buffer {
     		draw location + circle (vision) color: color ;
		}   
	
	reflex move_around {
		
		// do flock scenario
		
		if scenario = "flocking time" {
			
			grazed_grass <- one_of (pasture_cells overlapping self) ;
			action_area <- circle(cow_speed) intersection cone(heading - cow_amplitude/2, heading + cow_amplitude/2);
		
		// checking for cells with biomass before flocking 
			grazed_list <- pasture_cells overlapping action_area;
			
			if 	length (grazed_list)>3	
			{
				do flock;
			}	
			else 
			{
				do wander speed: cow_speed bounds:pasture_polygon;
			}					
			
		}
		
		//random walk
		if scenario = "random walk" {
			do wander speed: cow_speed bounds:pasture_polygon;
			grazed_grass <- one_of (pasture_cells overlapping self) ;
			action_area <- circle(cow_speed) ; 
		}
		//correlated random walk
		if scenario = "correlated random walk" {
			do wander amplitude: cow_amplitude speed: cow_speed bounds:pasture_polygon;
			action_area <- circle(cow_speed) intersection cone(heading - cow_amplitude/2, heading + cow_amplitude/2) ; 			
		}	
		//one cow moves with correlated random walk, the others follow this cow.	
		if scenario = "lead cow & followers" {
			if name != "cows0" {
				do goto target: cows[0] speed: cow_speed;
				action_area <- line([self.location, self.location + cows[0]]) intersection circle(cow_speed);
				action_area <- circle(cow_speed);  				
			}
			else {
				//do wander amplitude: cow_amplitude speed: cow_speed bounds:pasture_polygon;			
				action_area <- circle(cow_speed) intersection cone(heading - cow_amplitude/2, heading + cow_amplitude/2) ; 
			}
		}
		//each cow always moves to the spot within reach that has the highest biomass.	
		if scenario = "go to the spot with most grass" {
			action_area <- circle(cow_speed) ; 			
			grazed_grass <- (shuffle(pasture_cells overlapping action_area)) with_max_of each.biomass;

			location <- grazed_grass.location;
			action_area <- circle(cow_speed) ; 
		}	
		
		// Split movement 	
		
	}
	
	//viz of cow agent	
	aspect base {
		draw cow_visualization size:20 color: #red;
	}

	
 	aspect action_neighbourhood {
		draw action_area color: #orange ; 
 	}	
}


/* Build a Cellular Automaton (CA) labelled "pasture" with the dimensions of 10 by 10 cells 
 * and a Moore neighbourhood*/
grid grass neighbors:8 {
	float biomass <- 5.0;	 	
}

//run the simulation
experiment Simulation type:gui {
	parameter "Scenario" var: scenario <- "random walk" among: ["random walk","correlated random walk", "lead cow & followers", "flocking time","go to the spot with most grass"] ;
	parameter "Vision" var: vision;
	parameter "minimum separation" var:min_separation;
	parameter "maximum cohere turn" var:max_cohere_turn;
	parameter "maximum separate turn" var:max_separate_turn;
	parameter"maximum align turn" var:max_align_turn;
	
	output {
		display charts {
			//how much did each cow eat over time?
			chart "energy" type: series  position:{0,0} size:{1,0.5}{
				loop i from: 0 to: 9 step: 1 {
					data "cow" + i value: cows[i].energy color:#blue;
				}
			}
			//how much biomass is available on the pasture over time?
			chart "biomass" type: series position:{0,0.5} size:{1,0.5} {
				data "pasture" value:  sum(grass collect each.biomass) color:#green;
			}
			
		}			
		display "Vierkaser map" type:opengl {
			
			grid grass ;
			species cows aspect:base refresh:true ;
			species cows aspect:action_neighbourhood refresh:true transparency:0.5;	
			//species cows aspect:sheep_vis refresh:true transparency:0.5;	
		}
	}
}