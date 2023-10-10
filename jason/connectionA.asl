// Agent bob in project MAPC2018.mas2j

/* Initial beliefs and rules */
explorer_mode(1).
moving_to_goal(0).
last_action_result(none).
last_action(none).
my_field_of_view(M).
explored(0,0).
failed_path(0).
failed(0).
failed_forbidden(0).
failed_target(0).
attached(0).
requested(0).
task_queue(Q).
moving_to_block(0).


/* Initial goals */
!start.

/* Plans */
//QED
+!start : true  <-   
	//Initialize agent viewed cells maps
	// Objective function to minimize navigating around blocks that 
	// were already seen by the agent
	+factor(0);
	+my_position(0,0);
	.queue.create(Q);
	-+task_queue(Q);
	// Initially populate my_map with my range of vision
	// at the start of the simulation
    for ( .member(V,[5,4,3,2,1,0]) ) {
		for(.range(I,-V,V)){
			if(V == 5){
				?my_field_of_view(MyMap);
				.map.put(M1,I,0);
				.map.put(MyMap,M1,"T");
				-+my_field_of_view(MyMap);
				+explored(I,0);
			}
			elif(V < 5){
				?my_field_of_view(MyMap);
				.map.put(M2,I,(V - 5)* - 1);
				.map.put(M3,I,(V - 5));
				.map.put(MyMap,M2, "T");
				.map.put(MyMap,M3, "T");
				-+my_field_of_view(MyMap);
				+explored(I,(V - 5)* - 1);
				+explored(I,(V - 5));
			}
		}
	}
	.
	

//explore the map continously using and obejctive function 
//to minimize the number of grid cells that were visited by 
//the agent
+!explore(E): true <-
	//Probablisticly will move in a random direction 
	//Every 4 step to avoid obstacles
	
	//Use random exploration since the path planning algorithm
	//still gets stuck in some versions of the map with 
	//dense blocks and goals
	.random(R);
	 if(1==1){
		 .random(Rand);
		 RandomDir = Rand * 4;
		 if(RandomDir < 1){
		 	-+last_action(move_e);
		 	move(e);
		 }
		 if(RandomDir >= 1 & RandomDir < 2 ){
		 	-+last_action(move_w);
		 	move(w);
		 }
		 if(RandomDir >= 2 & RandomDir < 3){
		 	-+last_action(move_n);
		 	move(n);
		 }
		 if(RandomDir >= 3){
		 	-+last_action(move_s);
		 	move(s);
		 }
	 }
	 else{
		 ?my_field_of_view(MyMap);		 
		 ?my_position(X,Y);
		 .map.put(CoordMap,"East",0);
		 .map.put(CoordMap,"West",0);
		 .map.put(CoordMap,"North",0);
		 .map.put(CoordMap,"South",0);
		 //////////////////
		 for ( .map.key(MyMap,K) & .map.get(MyMap,K,V) ) { 
		 	for ( .map.key(K,XE) & .map.get(K,XE,YE) ) {
					.count(explored(XE+1,YE),Count);
		 			if(Count == 0){
					.map.get(CoordMap,"East",EastCount);
					 NewEastCount = EastCount + 1;
					.map.put(CoordMap,"East",NewEastCount);
				}
			}
			for ( .map.key(K,XW) & .map.get(K,XW,YW) ) {
					.count(explored(XW-1,YW),Count);
		 			if(Count == 0){
					.map.get(CoordMap,"West",WestCount);
					 NewWestCount = WestCount + 1;
					.map.put(CoordMap,"West",NewWestCount);
				}
			}
			for ( .map.key(K,XN) & .map.get(K,XN,YN) ) {
					.count(explored(XN,YN+1),Count);
		 			if(Count == 0){
					.map.get(CoordMap,"North",NorthCount);
					NewNorthCount = NorthCount + 1;
					.map.put(CoordMap,"North",NewNorthCount);
				}
			}
			for ( .map.key(K,XS) & .map.get(K,XS,YS) ) {
					.count(explored(XS,YS-1),Count);
		 			if(Count == 0){
					.map.get(CoordMap,"South",SouthCount);
					NewSouthCount = SouthCount + 1;
				}
			}
		 }
		//.print(CoordMap);
		.map.get(CoordMap,"East",EastCount);
		.map.get(CoordMap,"West",WestCount);
		.map.get(CoordMap,"North",NorthCount);
		.map.get(CoordMap,"South",SouthCount);
		.max([EastCount, WestCount, NorthCount, SouthCount], MAX);		
		
		?failed_path(FP);
		?failed_forbidden(FF);
		?failed_target(FT);
		?failed(F);
		?last_action(LA);
		
		if(LA == none){move(e);}
		if(LA == move_e & (FP == 1 | FF == 1 | FT == 1 | F == 1)){.map.put(CoordMap,"East",-100);}
		if(LA == move_w & (FP == 1 | FF == 1 | FT == 1 | F == 1)){.map.put(CoordMap,"West",-100);}
		if(LA == move_n & (FP == 1 | FF == 1 | FT == 1 | F == 1)){.map.put(CoordMap,"North",-100);}
		if(LA == move_s & (FP == 1 | FF == 1 | FT == 1 | F == 1)){.map.put(CoordMap,"South",-100);}
		
		if(Max == EastCount){
			if(LA == move_e & (FP == 1 | FF == 1) &  moving_to_goal(M) & M == 0){!move_west(M);}else{!move_east(M);}
		}
		elif(MAX == WestCount){
			if(LA == move_w & (FP == 1 | FF == 1) & moving_to_goal(M) & M == 0){!move_east(M);}else{!move_west(M);}
		}
		elif(MAX == NorthCount){
			if(LA == move_n & (FP == 1 | FF == 1) & moving_to_goal(M) & M == 0){!move_south(M);}else{!move_north(M);}
		}
		elif(MAX == SouthCount){
			if(LA == move_s & (FP == 1 | FF == 1) & moving_to_goal(M) & M == 0){!move_north(M);}else{!move_south(M);}
		}
	}.
	
+!move_east(M): true <-
			//Update agent beliefs, extend its explored teritorry by 1 block to the west
			?my_field_of_view(MyMap);		 
			?my_position(XE, YE);
			NewX = XE + 1;
			NewY = YE;
			-my_position(P1, P2);
			+my_position(NewX, NewY);
			.map.create(MyNewMap);
			for (.map.key(MyMap,KNew) & .map.get(MyMap,KNew,VNew) ) { 
				for ( .map.key(KNew,XNew) & .map.get(KNew,XNew,YNew) ) {
						UpdatedX = XNew + 1;
						UpdatedY = YNew;						
						.map.put(KUpdated,UpdatedX, UpdatedY);
						.map.put(MyNewMap,KUpdated, "U");
						+explored(UpdatedX,UpdatedY);
				}
			}
			
			move(e);
			-+my_field_of_view(MyNewMap);
			-+last_action(move_e);
.

+!move_west(M): true <-
			//Update agent beliefs, extend its explored teritorry by 1 block to the west
			?my_position(XW, YW);
			?my_field_of_view(MyMap);		 
			NewX = XW - 1;
			NewY = YW;
			-my_position(P1, P2);
			+my_position(NewX, NewY);
			.map.create(MyNewMap);
			for (.map.key(MyMap,KNew) & .map.get(MyMap,KNew,VNew) ) { 
				for ( .map.key(KNew,XNew) & .map.get(KNew,XNew,YNew) ) {
						UpdatedX = XNew - 1;
						UpdatedY = YNew;						
						.map.put(KUpdated,UpdatedX, UpdatedY);
						.map.put(MyNewMap,KUpdated, "U");
						+explored(UpdatedX,UpdatedY);
				}
			}
			
			move(w);
			-+my_field_of_view(MyNewMap);
			-+last_action(move_w);
.

+!move_north(M): true <-
			//Update agent beliefs, extend its explored teritorry by 1 block to the west
			?my_position(XN, YN);
			?my_field_of_view(MyMap);		 
			NewX = XN;
			NewY = YN - 1;
			-my_position(P1, P2);
			+my_position(NewX, NewY);
			.map.create(MyNewMap);
			for (.map.key(MyMap,KNew) & .map.get(MyMap,KNew,VNew) ) { 
				for ( .map.key(KNew,XNew) & .map.get(KNew,XNew,YNew) ) {
						UpdatedX = XNew;
						UpdatedY = YNew + 1;						
						.map.put(KUpdated,UpdatedX, UpdatedY);
						.map.put(MyNewMap,KUpdated, "U");
						+explored(UpdatedX,UpdatedY);
				}
			}
			
			move(n);
			-+my_field_of_view(MyNewMap);
			-+last_action(move_n);
.

+!move_south(M): true <-
			//Update agent beliefs, extend its explored teritorry by 1 block to the west
			?my_field_of_view(MyMap);		 
			?my_position(XS, YS);
			NewX = XS + 1;
			NewY = YS;
			-my_position(P1, P2);
			+my_position(NewX, NewY);
			.map.create(MyNewMap);
			for (.map.key(MyMap,KNew) & .map.get(MyMap,KNew,VNew) ) { 
				for ( .map.key(KNew,XNew) & .map.get(KNew,XNew,YNew) ) {
						UpdatedX = XNew - 1;
						UpdatedY = YNew;						
						.map.put(KUpdated,UpdatedX, UpdatedY);
						.map.put(MyNewMap,KUpdated, "U");
						+explored(UpdatedX,UpdatedY);
				}
			}
			
			move(s);
			-+my_field_of_view(MyNewMap);
			-+last_action(move_s);
.
//Continously explore the map until a dispenser is found 
//or a goal is found		
+step(Z) : explorer_mode (M) & M ==1<-
	!explore(M). 

//At each action in the simulation, if we saw the goal and we know
//the location of the grid cell in the goal center, we activate the
//goal !center_goal(X,Y) which instructs the agent to go the center grid cell	
+step(Z) : center_goal(X, Y) & moving_to_goal(M) & M == 1 <-
	!center_goal(X,Y);.
	
+step(Z) : explorer_mode(M) & M ==1 & clear_explored(yes)  <-
	
	for ( .member(V,[5,4,3,2,1,0]) ) {
		for(.range(I,-V,V)){
			if(V == 5){
				?my_field_of_view(MyMap);
				.map.put(M1,I,0);
				.map.put(MyMap,M1,"T");
				-+my_field_of_view(MyMap);
				+explored(I,0);
			}
			elif(V < 5){
				?my_field_of_view(MyMap);
				.map.put(M2,I,(V - 5)* - 1);
				.map.put(M3,I,(V - 5));
				.map.put(MyMap,M2, "T");
				.map.put(MyMap,M3, "T");
				-+my_field_of_view(MyMap);
				+explored(I,(V - 5)* - 1);
				+explored(I,(V - 5));
			}
		}
	}
	-clear_explored(yes);.

//In case we see a dispenser move towards it
+thing(X, Y, Z, L) : Z == dispenser & not (z == block)& X < 1 & requested(R) & R == 0 & moving_to_block(B) & B == 0<- 
	-+moving_to_block(1);
	move(w);
	-+last_action(move_w);.

//In case we see a dispenser move towards it	
+thing(X, Y, Z, L) : Z == dispenser & not (z == block) & X > 1 & requested(R) & R == 0 & moving_to_block(B) & B == 0<- 
	-+moving_to_block(1);
	move(e);
	-+last_action(move_e);.
	
//In case we see a dispenser move towards it
+thing(X, Y, Z, L) : Z == dispenser & not (z == block) & Y < 0 & requested(R) & R == 0 & moving_to_block(B) & B == 0<- 
	-+moving_to_block(1);
	move(n);
	-+last_action(move_b);.
	
//In case we see a dispenser move towards it
+thing(X, Y, Z, L) : Z == dispenser & not (z == block) & Y > 0 & requested(R) & R == 0 & moving_to_block(B) & B == 0<- 
	-+moving_to_block(1);
	move(s);
	-+last_action(move_s);.

//In case we have the block attached to the agent from its eastern side
//we rotate CW to get the block to the southern side
+thing(X, Y, Z, L) : Z == block & X == 1 & Y == 0 & attached(A) & A == 1 <- 
	rotate(cw).
	
//attempt to request block	
+thing(X, Y, Z, L) : Z == dispenser & X == 1 & Y == 0 & requested(R) & R == 0<-
	request(e);
	-+requested(1);
	-+moving_to_block(0);.

// In case my block is at my east rotate CW to ensure it is at my south
//+thing(X,Y,E,T): E == block & X \== 0 <-
	//.print("block is at right");
	//.
// attempt to attach block	
+thing(X, Y, Z, L) : attached(A) & A == 0 & requested(R) & R == 1<- 
	attach(e);
	-+attached(1);
	+clear_explored(yes);
	-+explorer_mode(1);
	.


//When a goal percept is received, wait until we can preceive
//2 grid cells and compare them adjacent cells will always have
//PY-Y == 0
//This function identitfies the center grid cell of the goal	
+goal(X, Y): attached(A) & A == 1 <-
	.count(potential_goal(PotX, PotY),Count);
	if(Count == 0){
		+potential_goal(X, Y);
	}
	else{
		?potential_goal(PX, PY);
		if(((PX-X) == 1) & (PY-Y) == 0 & moving_to_goal(M) & M == 0){
			// found the goal center to my west
			-+moving_to_goal(1);
			!center_goal(X, Y);
		}
		if(((PX-X) == -1) & (PY-Y) == 0 & moving_to_goal(M) & M == 0){
			// found the goal center to my east
			-+moving_to_goal(1);
			!center_goal(X, Y);
		}
		if(((PX-X) == 0) & (PY-Y) == 1 & moving_to_goal(M) & M == 0){
			// found the goal center to my west
			-+moving_to_goal(1);
			!center_goal(X, Y);
		}
		if(((PX-X) == 0) & (PY-Y) == -1 & moving_to_goal(M) & M == 0){
			// found the goal center to my west
			-+moving_to_goal(1);
			!center_goal(X, Y);
		}
	}.	


@c[atomic]	
+!center_goal(X,Y): moving_to_goal(M) & M == 1 & attached(A) & A == 1<-
	-+explorer_mode(0);
	
	if(X<0){
	 	move(w);
		-+last_action(move_w);
		-+center_goal(X+1,Y);
	}
	elif(X>0){
	 	move(e);
		-+last_action(move_e);
		-+center_goal(X-1,Y);
	}
	elif(Y<0){
	 	move(n);
		-+last_action(move_n);
		-+center_goal(X,Y+1);
	}
	elif(Y>0){
	 	move(s);
		-+last_action(move_s);
		-+center_goal(X,Y-1);
	}
	
	
	if(X == 0 & Y == 0){
		?task_queue(Q);
		.queue.head(Q,H);                // H = a
		.queue.remove(Q,H);              // H = a, Q = [b, c, d, e]
		submit(H);
		submit(H);
		submit(H);
		-+explorer_mode(1);
		-+moving_to_goal(0);
		-+attached(0);
		-center_goal(M, N);
		-potential_goal(L,K);
	}
	//-moving_to_goal(yes);
	//+explorer_mode(yes);
	.

@s[atomic]
+!submit(A): true <-
	.print("submitting");
	?task_queue(Q);
		.queue.head(Q,H);                // H = a
		.queue.remove(Q,H);              // H = a, Q = [b, c, d, e]
		submit(H);
		submit(H);
		submit(H);
	-+explorer_mode(1);
	-+moving_to_goal(0);
	-+attached(0);
	-center_goal(M, N);
	-potential_goal(L,K);               
.

+task(N, D, R, List) : .length(List,Length) & Length == 1 <-
	.count(target_task(A),Count);
	?task_queue(Q);
	if(R == 10 & Count == 0){
	.queue.add(Q,N);
		-+task_queue(Q);
	}
	.
+failed_target(T) : not clear_explored(yes) <-
	+clear_explored(yes);
	+explorer_mode(1);
	.
+lastActionResult(Result): true <-
	-+last_action_result(Result);
	if(Result == failed_path){
		-+failed_path(1);
		-+explorer_mode(1);
		-+moving_to_goal(0);
	}
	if(Result == failed_forbidden){
		-+failed_forbidden(1);
		-+explorer_mode(1);
		-+moving_to_goal(0);
	}
	if(Result == failed_target){
		-+failed_target(1);
		-+explorer_mode(1);
		-+moving_to_goal(0);
		-center_goal(M, N);
		-potential_goal(L,K);
	}
	if(Result == failed){
		-+failed(1);
		-+explorer_mode(1);
	}
	//.print(Result);
	.
