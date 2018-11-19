;;; This file contains a SHOP domain representation of the block-stacking
;;; algorithm from the following paper:
;;;    N. Gupta and D. Nau, On the complexity of blocks-world planning,
;;;    Artificial Intelligence 56(2-3):223-254, August 1992.


;;; ------------------------------------------------------------------------
;;; Declare all the data
;;; ------------------------------------------------------------------------

(defdomain colregs
    (

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; The method for sailling between two waypoints
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (:method (sail ?boat ?boat_head ?gc)

            sail1_LastWaypointReached
            (;;precond
                ;; Reached last waypoint

                (at             ?boat ?gc   )
                (waypoint-last  ?gc         )
            )                                     
            ();; tasks network

            sail2_NotEnd_WayFree
            (;;precond   
                ;; NOT reached last waypoint, way is free (no obstacle)

                (at                     ?boat       ?gc             )
                (NOT(waypoint-last      ?gc)                        )
                (way-free               ?boat                       )
                (adj                    ?boat_head  ?adj_gc     ?gc )           ;; gets adj_gc
            )                    
            (;; tasks network
                ;; Verify collision and go to next detected free position

                (collision-detection    ?boat       ?boat_head          )
                (goto-next-position     ?boat       ?gc                 )
                (sail                   ?boat       ?boat_head  ?adj_gc )
            )     

            sail3_NotEnd_ObstacleDetected
            (;;precond
                ;; NOT reached last waypoint, obstacle detected

                (at                     ?boat       ?gc             )
                (NOT(waypoint-last      ?gc)                        )
                (NOT(way-free           ?boat)                      )
                (adj                    ?boat_head  ?adj_gc     ?gc ) ;; sai essa precondicap
            )               
            (;; tasks network
                (colregs-decision       ?boat       ?boat_head          )
                (sail                   ?boat       ?boat_head  ?adj_gc ) ;; passa a posicao atual
            )
            
        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; collision-detection method
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
        (:method (collision-detection ?boat ?boat_head)

            ;; Intruder on radius
            intruder-on-danger-zone
            (;;precond   
                (at ?boat   ?gc)                ;; Discovers boat's current grid cell
                (OR                             ;; Test if any of the 2 steps adjascent grid cells are occupied
                    (OR 
                        (
                            (adj E  ?adj_gc     ?gc         )   ;; Gets EAST adjascent grid 
                            (at     ?intruder   ?adj_gc     )   ;; Tests if the adjascent grid is occupied
                        )
                        (
                            (adj E  ?adj_gc     ?gc         )   ;; Gets EAST adjascent grid 
                            (adj E  ?adj_gc2    ?adj_gc     )   ;; Gets EAST EAST 
                            (at     ?intruder   ?adj_gc2    )   ;; Tests if the adjascent grid is occupied 
                        )                        
                    )
                    (OR 
                        (
                            (adj SE ?adj_gc     ?gc         )   ;; Gets SOUTH EAST adjascent grid  
                            (at     ?intruder   ?adj_gc     )   ;; Tests if the adjascent grid is occupied
                        )
                        (
                            (adj SE ?adj_gc     ?gc         )   ;; Gets SOUTH EAST adjascent grid  
                            (adj E  ?adj_gc2    ?adj_gc     )   ;; Gets SOUTH EAST EAST adjascent grid  
                            (at     ?intruder   ?adj_gc2    )   ;; Tests if the adjascent grid is occupied
                        )
                        (
                            (adj SE ?adj_gc     ?gc         )   ;; Gets SOUTH EAST adjascent grid                              
                            (adj SE ?adj_gc2    ?adj_gc     )   ;; Gets SOUTH EAST SOUTH EAST adjascent grid  
                            (at     ?intruder   ?adj_gc2    )   ;; Tests if the adjascent grid is occupied
                        )
                        (   
                            (adj SE ?adj_gc     ?gc         )   ;; Gets SOUTH EAST adjascent grid  
                            (adj S  ?adj_gc2    ?adj_gc     )   ;; Gets SOUTH EAST SOUTH adjascent grid  
                            (at     ?intruder   ?adj_gc2    )   ;; Tests if the adjascent grid is occupied                   )
                        )
                    )
                    ;;(OR   
                    ;;    (adj S  ?adj_gc     ?gc         )         ;; Gets SOUTH adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj S  ?adj_gc2    ?adj_gc     )         ;; Gets SOUTH SOUTH adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                    ;;(OR   
                    ;;    (adj SW ?adj_gc     ?gc         )        ;; Gets SOUTH WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj S  ?adj_gc2    ?adj_gc     )        ;; Gets SOUTH WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj SW ?adj_gc2    ?adj_gc     )        ;; Gets SOUTH WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj W  ?adj_gc2    ?adj_gc     )        ;; Gets SOUTH WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                    ;;(OR   
                    ;;    (adj W  ?adj_gc     ?gc         )         ;; Gets WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj W  ?adj_gc2    ?adj_gc     )         ;; Gets WEST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                    ;;(OR   
                    ;;    (adj NW ?adj_gc     ?gc         )        ;; Gets NORTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj W  ?adj_gc2    ?adj_gc     )        ;; Gets NORTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj NW ?adj_gc2    ?adj_gc     )        ;; Gets NORTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj N  ?adj_gc2    ?adj_gc     )        ;; Gets NORTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                    ;;(OR   
                    ;;    (adj N  ?adj_gc     ?gc         )         ;; Gets NORTH adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj N  ?adj_gc2    ?adj_gc     )         ;; Gets NORTH adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                    ;;(OR   
                    ;;    (adj NE ?adj_gc     ?gc         )        ;; Gets NORTTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc     )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj N  ?adj_gc2    ?adj_gc     )        ;; Gets NORTTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj NE ?adj_gc2    ?adj_gc     )        ;; Gets NORTTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;    (adj E  ?adj_gc2    ?adj_gc     )        ;; Gets NORTTH EAST adjascent grid  
                    ;;    (at     ?intruder   ?adj_gc2    )      ;; Tests if the adjascent grid is occupied
                    ;;)
                )
            )
            (;; tasks network
                (!operator collision-detected ?boat)
            )

            no-intruder-on-danger-zone
            ();;precond
            ();;task network

        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; collision-detected operator
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (:operator (!collision-detected ?boat)
            ();;precond
            (;;delete list            
                (way-free ?boat)
            )
            ();;add list

        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; goto-next-position method
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (:method (goto-next-position ?boat ?gc)

            move
            (;;precond
                (boat_head  ?boat   ?dir        )   ;; Discovers boat's direction
                (adj        ?dir    ?adj_gc ?gc )   ;; Discovers adjacent grid cell
                (way-free   ?boat               )   ;; Moves only if there is no obstacle detected
            )
            (;; tasks network
                (!goto      ?boat   ?gc     ?adj_gc)
            )
            

        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; goto operator
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (:operator (!goto ?boat ?gc ?adj_gc)
            ();;precond
            (;;delete list
                (at         ?boat   ?gc     )      
            )
            (;;add list
                (at         ?boat   ?adj_gc )
            )
        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; colregs-decision method
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
        (:method (colregs-decision ?boat ?boat_head)

            head-on
            (;;precond
                (OR
                    (
                        (at     ?boat       ?gc                 )   ;; Discovers boat's current grid cell
                        (adj    ?boat_head  ?adj_gc ?gc         )   ;; Discovers adjascent grid explicity ahead
                        (at     ?intruder   ?adj_gc             )   ;; Discovers if adjascent grid explicity ahead is occupied
                    )
                    (   
                        (at     ?boat       ?gc                 )   ;; Discovers boat's current grid cell
                        (adj    ?boat_head  ?adj_gc ?gc         )   ;; Discovers adjascent grid explicity ahead
                        (adj    ?boat_head  ?adj_gc_2 ?adj_gc   )   ;; Discovers adjascent grid 2 steps ahead
                        (at     ?intruder   ?adj_gc_2           )   ;; Discovers if adjascent grid 2 steps ahead is occupied
                    )
                )   
                    
                    
            )
            (;; tasks network
                (!operator head-on-detected boat boat_head)
            )

            crossing-starboard
            (;;precond   
                (at ?boat   ?gc)                ;; Discovers boat's current grid cell
                
                (OR                             ;; Discovers boat's adjascent grid cell
                    (   
                        (boat_head ?boat E)     ;; current EAST
                        (adj SE ?adj_gc ?gc)    ;; adjascent SE
                    )
                    (   
                        (boat_head ?boat S)     ;; current SOUTH
                        (adj SW ?adj_gc ?gc)    ;; adjascent SW
                    )
                    (   
                        (boat_head ?boat W)     ;; current WEST
                        (adj NW ?adj_gc ?gc)    ;; adjascent NW
                    )
                    (   
                        (boat_head ?boat N)     ;; current NORTH
                        (adj NE ?adj_gc ?gc)    ;; adjascent NE
                    )
                
                )
                (at ?intruder ?adj_gc)          ;; Discovers if adjascent grid is occupied

                (OR                             ;; Discovers boat's adjascent grid cell
                    (   
                        ;; Test specific boat's orientation and adjascency occupancy
                        (boat_head ?boat E)                
                        (adj SE ?adj_gc_2 ?adj_gc)      ;; Discovers 2 steps adjascent grid cell                
                    )
                    (   
                        ;; Test specific boat's orientation and adjascency occupancy
                        (boat_head ?boat S)                
                        (adj SW ?adj_gc_2 ?adj_gc)      ;; Discovers 2 steps adjascent grid cell            
                    )
                    (   
                        ;; Test specific boat's orientation and adjascency occupancy
                        (boat_head ?boat W)                
                        (adj NW ?adj_gc_2 ?adj_gc)      ;; Discovers 2 steps adjascent grid cell
                    )
                    (   
                        ;; Test specific boat's orientation and adjascency occupancy
                        (boat_head ?boat N)                
                        (adj NE ?adj_gc_2 ?adj_gc)      ;; Discovers 2 steps adjascent grid cell
                    )
                
                )

                (adj ?boat_head ?adj_gc_2 ?adj_gc)      ;; Discovers adjascent grid 2 steps ahead
                (at ?intruder ?adj_gc_2)                ;; Discovers if adjascent grid 2 steps ahead is occupied
            )
            (;; tasks network
                (!operator collision-detected boat boat_head)
            )

        )

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; head-on-detected
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
        (:operator (!head-on-detected ?boat ?boat_head)
            (;;precond
                (at        ?boat   ?gc)             ;; Discovers grid cell occupied by main vessel
                (OR                                 ;; Discovers boat's adjascent grid cell that is at its starboard side
                    (   
                        (boat_head  ?boat E)        ;; current EAST
                        (adj SE     ?adj_gc ?gc)    ;; starboard at: SE
                    )
                    (   
                        (boat_head  ?boat S)        ;; current SOUTH
                        (adj SW     ?adj_gc ?gc)    ;; starboard at: SW
                    )
                    (   
                        (boat_head  ?boat W)        ;; current WEST
                        (adj NW     ?adj_gc ?gc)    ;; starboard at: NW
                    )
                    (   
                        (boat_head  ?boat N)        ;; current NORTH
                        (adj NE     ?adj_gc ?gc)    ;; starboard at: NE
                    )
                
                )
            )
            (;;delete list
                (way-free   ?boat)
                (at         ?boat   ?gc)
            )
            (;;add list
                (at         ?boat   ?adj_gc)
            )
        )
    )
)
