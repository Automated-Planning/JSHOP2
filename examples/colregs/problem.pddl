(defproblem problem colregs
    (
        ;;;
        ;;;  facts
        ;;;



        ;; grid cells definition
        (GRID_CELL GC_A1)  (GRID_CELL GC_B1)  (GRID_CELL GC_C1)  
        (GRID_CELL GC_A2)  (GRID_CELL GC_B2)  (GRID_CELL GC_C2)  
        (GRID_CELL GC_A3)  (GRID_CELL GC_B3)  (GRID_CELL GC_C3)  
        (GRID_CELL GC_A4)  (GRID_CELL GC_B4)  (GRID_CELL GC_C4)  
        (GRID_CELL GC_A5)  (GRID_CELL GC_B5)  (GRID_CELL GC_C5)  
        (GRID_CELL GC_A6)  (GRID_CELL GC_B6)  (GRID_CELL GC_C6)  
        (GRID_CELL GC_A7)  (GRID_CELL GC_B7)  (GRID_CELL GC_C7)


        ;; Adjascency definition         
        ;;;; First line
        ;;;;;;;; GC_A1                     
                                (ADJ E  GC_A2 GC_A1)  
        (ADJ S  GC_B1 GC_A1)    (ADJ SE GC_B2 GC_A1)                        
        ;;;;;;;; GC_A2
        (ADJ W  GC_A1 GC_A2)                            (ADJ E  GC_A3 GC_A2)
        (ADJ SW GC_B1 GC_A2)    (ADJ S  GC_B2 GC_A2)    (ADJ SE GC_B3 GC_A2)
        ;;;;;;;; GC_A3
        (ADJ W  GC_A2 GC_A3)                            (ADJ E  GC_A4 GC_A3)
        (ADJ SW GC_B2 GC_A3)    (ADJ S  GC_B3 GC_A3)    (ADJ SE GC_B4 GC_A3)
        ;;;;;;;; GC_A4
        (ADJ W  GC_A3 GC_A4)                            (ADJ E  GC_A5 GC_A4)
        (ADJ SW GC_B3 GC_A4)    (ADJ S  GC_B4 GC_A4)    (ADJ SE GC_B5 GC_A4)
        ;;;;;;;; GC_A5
        (ADJ W  GC_A4 GC_A5)                            (ADJ E  GC_A6 GC_A5)
        (ADJ SW GC_B4 GC_A5)    (ADJ S  GC_B5 GC_A5)    (ADJ SE GC_B6 GC_A5)
        ;;;;;;;; GC_A6
        (ADJ W  GC_A5 GC_A6)                            (ADJ E  GC_A7 GC_A6)
        (ADJ SW GC_B5 GC_A6)    (ADJ S  GC_B6 GC_A6)    (ADJ SE GC_B7 GC_A6)
        ;;;;;;;; GC_A7
        (ADJ W  GC_A6 GC_A7)
        (ADJ S  GC_B7 GC_A7)    (ADJ SW GC_B6 GC_A7)



        ;;;; Second line
        ;;;;;;;; GC_B1             
        (ADJ N  GC_A1 GC_B1)    (ADJ NE GC_A2 GC_B1)  
                                (ADJ E  GC_B2 GC_B1)  
        (ADJ S  GC_C1 GC_B1)    (ADJ SE GC_C2 GC_B1)  
                                            
        ;;;;;;;; GC_B2
        (ADJ NW GC_A1 GC_B2)    (ADJ N  GC_A2 GC_B2)    (ADJ NE GC_A3 GC_B2)
        (ADJ W  GC_B1 GC_B2)                            (ADJ E  GC_B3 GC_B2)
        (ADJ SW GC_C1 GC_B2)    (ADJ S  GC_C2 GC_B2)    (ADJ SE GC_C3 GC_B2)
 
        ;;;;;;;; GC_B3
        (ADJ NW GC_A2 GC_B3)    (ADJ N  GC_A3 GC_B3)    (ADJ NE GC_A4 GC_B3)
        (ADJ W  GC_B2 GC_B3)                            (ADJ E  GC_B4 GC_B3)
        (ADJ SW GC_C2 GC_B3)    (ADJ S  GC_C3 GC_B3)    (ADJ SE GC_C4 GC_B3)
        
        ;;;;;;;; GC_B_4
        (ADJ NW GC_A3 GC_B4)    (ADJ N  GC_A4 GC_B4)    (ADJ NE GC_A5 GC_B4)
        (ADJ W  GC_B3 GC_B4)                            (ADJ E  GC_B5 GC_B4)
        (ADJ SW GC_C3 GC_B4)    (ADJ S  GC_C4 GC_B4)    (ADJ SE GC_C5 GC_B4)
        
        ;;;;;;;; GC_B5
        (ADJ NW GC_A4 GC_B5)    (ADJ N  GC_A5 GC_B5)    (ADJ NE GC_A6 GC_B5)
        (ADJ W  GC_B4 GC_B5)                            (ADJ E  GC_B6 GC_B5)
        (ADJ SW GC_C4 GC_B5)    (ADJ S  GC_C5 GC_B5)    (ADJ SE GC_C6 GC_B5)
                
        ;;;;;;;; GC_B6
        (ADJ NW GC_A5 GC_B6)    (ADJ N  GC_A6 GC_B6)    (ADJ NE GC_A7 GC_B6)
        (ADJ W  GC_B5 GC_B6)                            (ADJ E  GC_B7 GC_B6)
        (ADJ SW GC_C5 GC_B6)    (ADJ S  GC_C6 GC_B6)    (ADJ SE GC_C7 GC_B6)
                
        ;;;;;;;; GC_B7
        (ADJ NW GC_A6 GC_B7)    (ADJ N  GC_A7 GC_B7)
        (ADJ W  GC_B6 GC_B7)                      
        (ADJ SW GC_C6 GC_B7)    (ADJ S  GC_C7 GC_B7)

        ;;;; Third line
        ;;;;;;;; GC_C1
        (ADJ N  GC_B1 GC_C1)    (ADJ NE GC_B2 GC_C1)
                                (ADJ E  GC_C2 GC_C1)
 
        ;;;;;;;; GC_C2
        (ADJ NW GC_B1 GC_C2)    (ADJ N  GC_B2 GC_C2)    (ADJ NE GC_B3 GC_C2)
        (ADJ W  GC_C1 GC_C2)                            (ADJ E  GC_C3 GC_C2)

        ;;;;;;;; GC_C3
        (ADJ NW GC_B2 GC_C3)    (ADJ N  GC_B3 GC_C3)    (ADJ NE GC_B4 GC_C3)
        (ADJ W  GC_C2 GC_C3)                            (ADJ E  GC_C4 GC_C3)

        ;;;;;;;; GC_C4
        (ADJ NW GC_B3 GC_C4)    (ADJ N  GC_B4 GC_C4)    (ADJ NE GC_B5 GC_C4)
        (ADJ W  GC_C3 GC_C4)                            (ADJ E  GC_C5 GC_C4)
 
        ;;;;;;;; GC_C5
        (ADJ NW GC_B4 GC_C5)    (ADJ N  GC_B5 GC_C5)    (ADJ NE GC_B6 GC_C5)
        (ADJ W  GC_C4 GC_C5)                            (ADJ E  GC_C6 GC_C5)

        ;;;;;;;; GC_C6
        (ADJ NW GC_B5 GC_C6)    (ADJ N  GC_B6 GC_C6)    (ADJ NE GC_B7 GC_C6)
        (ADJ W  GC_C5 GC_C6)                            (ADJ E  GC_C7 GC_C6)

        ;;;;;;;; GC_C7
        (ADJ NW GC_B6 GC_C6)    (ADJ N  GC_B7 GC_C7)
        (ADJ W  GC_C6 GC_C6)
        

        ;; waypoints
        ;;(waypoint       GC_B1)
        ;;(waypoint       GC_B2)
        ;;(waypoint       GC_B3)
        ;;(waypoint       GC_B4)
        ;;(waypoint       GC_B5)
        ;;(waypoint       GC_B6)
        ;;(waypoint       GC_B7)
        (waypoint-last    GC_A7)
        (waypoint-last    GC_B7)
        (waypoint-last    GC_C7)

        ;;;
        ;;;  initial states
        ;;;

        (boat       boat1                   )
        (at         boat1       GC_B1       )
        (boat_head  boat1       E           )
        (way-free    boat1                  )

        (boat       intruder                )
        (at         intruder    GC_B3       )        
    )
    (   ;; GOALS
        (sail   boat1       E) 
    )
)