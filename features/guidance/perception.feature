@routing  @guidance @perceived-angles
Feature: Simple Turns

    Background:
        Given the profile "car"
        Given a grid size of 5 meters

    Scenario: Turning into splitting road
        Given the node map
            """
              a
              b
              ^
             / \
            c   d
                |\
                | e
                |
                f
            """

        And the ways
            | nodes | name | highway | oneway |
            | ab    | road | primary | no     |
            | bc    | road | primary | yes    |
            | fdb   | road | primary | yes    |
            | de    | turn | primary | no     |

        When I route I should get
            | waypoints | turns                           | route          |
            | f,a       | depart,arrive                   | road,road      |
            | e,a       | depart,turn slight right,arrive | turn,road,road |

    Scenario: Turning into splitting road
        Given the node map
        """
              a
            g-b
              /\
             /  \
            c   d
                |\
                | e
                |
                f
        """

        And the ways
            | nodes | name | highway | oneway |
            | ab    | road | primary | no     |
            | bc    | road | primary | yes    |
            | fdb   | road | primary | yes    |
            | de    | turn | primary | no     |
            | bg    | left | primary | yes    |

        When I route I should get
            | waypoints | turns                                     | route               |
            | f,a       | depart,arrive                             | road,road           |
            | e,a       | depart,turn slight right,arrive           | turn,road,road      |
            | e,g       | depart,turn slight right,turn left,arrive | turn,road,left,left |
            | f,g       | depart,turn left,arrive                   | road,left,left      |
            | f,c       | depart,continue uturn,arrive              | road,road,road      |

    Scenario: Middle Island
        Given the node map
            """
              a

              b
            c   h




            d   g
              e

              f
            """

        And the ways
            | nodes | name | oneway |
            | ab    | road | no     |
            | ef    | road | no     |
            | bcde  | road | yes    |
            | eghb  | road | yes    |

        When I route I should get
            | waypoints | turns         | route     |
            | a,f       | depart,arrive | road,road |
            | c,f       | depart,arrive | road,road |
            | f,a       | depart,arrive | road,road |
            | g,a       | depart,arrive | road,road |

    Scenario: Middle Island Over Bridge
        Given the node map
            """
              a
              |
             .b.
            c   h
            |   |
            |   |
            1   2
            |   |
            d   g
             'e'
              |
              f
            """

        And the ways
            | nodes | name   | oneway |
            | ab    | road   | no     |
            | ef    | road   | no     |
            | bc    | road   | yes    |
            | cd    | bridge | yes    |
            | de    | road   | yes    |
            | eg    | road   | yes    |
            | gh    | bridge | yes    |
            | hb    | road   | yes    |

        When I route I should get
            | waypoints | turns                           | route            |
            | a,f       | depart,arrive                   | road,road        |
            | c,f       | depart,new name straight,arrive | bridge,road,road |
            | 1,f       | depart,new name straight,arrive | bridge,road,road |
            | f,a       | depart,arrive                   | road,road        |
            | g,a       | depart,new name straight,arrive | bridge,road,road |
            | 2,a       | depart,new name straight,arrive | bridge,road,road |

    @negative
    Scenario: Don't Collapse Places:
        Given the node map
            """
                        h
                        g




            a b                   e f




                        c
                        d
            """

        And the ways
            | nodes | name   | oneway |
            | ab    | place  | no     |
            | cd    | bottom | no     |
            | ef    | place  | no     |
            | gh    | top    | no     |
            | bcegb | place  | yes    |

        When I route I should get
            | waypoints | turns                                             | route                      |
            | a,d       | depart,turn right,arrive                          | place,bottom,bottom        |
            | a,f       | depart,continue left,continue right,arrive        | place,place,place,place    |
            | d,f       | depart,turn right,continue right,arrive           | bottom,place,place,place   |
            | d,h       | depart,turn right,continue left,turn right,arrive | bottom,place,place,top,top |

    @bug @not-sorted @3179
    Scenario: Adjusting road angles to not be sorted
        Given the node map
            """
                                 g
                                |
                               |
                              |
                             _e - - - - - - - - - f
                           /
            a - - - - -b <
                     i     \ _
                h             c - - - - - - - - - d

            """

        And the ways
            | nodes | name  | oneway |
            | ab    | road  | no     |
            | febcd | road  | yes    |
            | ge    | in    | yes    |
            | eh    | right | yes    |
            | ei    | left  | yes    |

        When I route I should get
            | waypoints | route        |
            | g,a       | in,road,road |

    @negative
    Scenario: Traffic Circle
        Given the node map
            """
            a - - - - b - .    e     . - c - - - - d
                        \     '    '   /
                            \      /
                               f
                               |
                               |
                               |
                               g
            """

        And the ways
            | nodes | name   | oneway |
            | ab    | left   | no     |
            | bfceb | circle | yes    |
            | fg    | bottom | no     |
            | cd    | right  | no     |

        When I route I should get
            | waypoints | route                           | intersections                                                                                       |
            | a,d       | left,circle,circle,right,right  | true:90;false:105 true:135 false:270;true:60 true:180 false:315;true:90 false:240 true:255;true:270 |
            | g,d       | bottom,circle,right,right       | true:0;true:60 false:180 false:315;true:90 false:240 true:255;true:270                              |


    Scenario: Traffic Island
        Given the node map
            """
                      f
            a - - b <   > d - - e
                      c
            """

        And the ways
            | nodes | name | oneway |
            | ab    | road | no     |
            | de    | road | no     |
            | bcdfb | road | yes    |

        When I route I should get
            | waypoints | route     | intersections    |
            | a,e       | road,road | true:90;true:270 |

    @negative
    Scenario: Turning Road, Don't remove sliproads
        Given the node map
            """
            h - - - - - g - - - - - - f - - - - - e
                               _   '
                           .
            a - - - - - b - - - - - - c - - - - - d
                        |
                        |
                        |
                        i
            """

        And the ways
            | nodes | name | oneway |
            | abcd  | road | yes    |
            | efgh  | road | yes    |
            | fb    | road | yes    |
            | bi    | turn | yes    |

        When I route I should get
            | waypoints | route          | intersections                                                                  |
            | a,d       | road,road      | true:90,false:60 true:90 true:180 false:270;true:270                           |
            | e,h       | road,road      | true:270,false:90 true:240 true:270;true:90                                    |
            | e,i       | road,turn,turn | true:270;false:90 true:240 true:270,false:60 true:90 true:180 false:270;true:0 |

     @negative
     Scenario: Meeting Turn Roads
        Given the node map
            """
                        k               l
                        |               |
                        |               |
                        |               |
            h - - - - - g - - - - - - - f - - - - - e
                        |   '        '  |
                        |       x       |
                        |   .        .  |
            a - - - - - b - - - - - - - c - - - - - d
                        |               |
                        |               |
                        |               |
                        i               j
            """

        And the ways
            | nodes | name  | oneway |
            | ab    | horiz | yes    |
            | bc    | horiz | yes    |
            | cd    | horiz | yes    |
            | ef    | horiz | yes    |
            | fg    | horiz | yes    |
            | gh    | horiz | yes    |
            | kg    | vert  | yes    |
            | gb    | vert  | yes    |
            | bi    | vert  | yes    |
            | jc    | vert  | yes    |
            | cf    | vert  | yes    |
            | fl    | vert  | yes    |
            | gx    | horiz | no     |
            | xc    | horiz | no     |
            | fx    | horiz | no     |
            | xb    | horiz | no     |

        And the relations
            | type        | way:from | way:to | node:via | restriction  |
            | restriction | bc       | cf     | c        | no_left_turn |
            | restriction | fg       | gb     | g        | no_left_turn |
            | restriction | cf       | fg     | f        | no_left_turn |
            | restriction | gb       | bc     | b        | no_left_turn |
            | restriction | xb       | bc     | b        | no_left_turn |
            | restriction | xc       | cf     | c        | no_left_turn |
            | restriction | xf       | fg     | f        | no_left_turn |
            | restriction | xg       | gb     | g        | no_left_turn |

        # the goal here should be not to mention the intersection in the middle at all
        When I route I should get
            | waypoints | route            | intersections                                                                                              |
            | a,l       | horiz,vert,vert  | true:90;false:0 true:60 true:90 true:180 false:270, true:0 false:90 false:180 false:240 false:270;true:180 |
            | a,d       | horiz,horiz      | true:90,false:0 true:60 true:90 true:180 false:270,false:0 true:90 false:180 false:270 true:300;true:270   |
            | j,h       | vert,horiz,horiz | true:0;true:0 true:90 false:180 false:270 true:300,false:0 false:90 false:120 false:180 true:270;true:90   |
            | j,l       | vert,vert        | true:0,true:0 true:90 false:180 false:270 true:300,true:0 false:90 false:180 true:240 false:270;true:180   |
