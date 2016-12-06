@guidance @merge-segregated
Feature: Merge Segregated Roads

    Background:
        Given the profile "car"
        Given a grid size of 3 meters

    #http://www.openstreetmap.org/#map=18/52.49950/13.33916
    @negative
    Scenario: oneway link road
        Given the node map
            """
            f - - - - - - -_-_e - - - - d
                      ...''
            a - - - b'- - - - - - - - - c
            """

        And the ways
            | nodes | name | oneway |
            | abc   | road | yes    |
            | def   | road | yes    |
            | be    | road | yes    |

        When I route I should get
            | waypoints | route     | intersections                                |
            | a,c       | road,road | true:90,true:75 true:90 false:270;true:270   |
            | d,f       | road,road | true:270,false:90 false:255 true:270;true:90 |

    #http://www.openstreetmap.org/#map=18/52.48337/13.36184
    @negative
    Scenario: Square Area - Same Name as road for in/out
        Given the node map
            """
                                  i
                                  |
                                  |
                                  |
                                  g
                                /   \
                              /       \
                            /           \
                          /               \
                        /                   \
            a - - - - c                       e - - - - f
                        \                   /
                          \               /
                            \           /
                              \       /
                                \   /
                                  d
                                  |
                                  |
                                  |
                                  j
            """

        And the ways
            | nodes | name | oneway |
            | ac    | road | no     |
            | ef    | road | no     |
            | cdegc | road | yes    |
            | ig    | top  | no     |
            | jd    | bot  | no     |

        When I route I should get
            | waypoints | route               | intersections                                                                                      |
            | a,f       | road,road,road,road | true:90,false:45 true:135 false:270;true:45 true:180 false:315;true:90 false:225 true:315;true:270 |

    #https://www.openstreetmap.org/#map=19/52.50003/13.33915
    @negative
    Scenario: Short Segment due to different roads
        Given the node map
            """
                                              . d
                                          . '
                                      . '
                                  . '
                              . '
            a - - - - - - - b - - c - - - - - - e
                            .     .
                            .     .
                             .   .
                              . .
                               .
                               f
                               |
                               |
                               |
                               |
                               g
            """

        And the ways
            | nodes | name | oneway |
            | abce  | pass | no     |
            | db    | pass | yes    |
            | fg    | aug  | no     |
            | bfc   | aug  | yes    |

        When I route I should get
            | waypoints | route     | intersections                                                                    |
            | a,e       | pass,pass | true:90,false:60 true:90 true:165 false:270,true:90 false:195 false:270;true:270 |

    @negative
    Scenario: Tripple Merge should not be possible
        Given the node map
            """
                          . f - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - g
                        .
            a - - - - b - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - e
                        '
                          ' c - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - d
            """

        And the ways
            | nodes | name  | oneway |
            | ab    | in    | no     |
            | gfb   | merge | yes    |
            | be    | merge | yes    |
            | dcb   | merge | yes    |

        When I route I should get
            | waypoints | route          | intersections                                          |
            | a,e       | in,merge,merge | true:90;false:75 true:90 false:120 false:270;true:270  |

    Scenario: Tripple Merge should not be possible
        Given the node map
            """
                          . f - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - g
                        .
            a - - - - b - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - e
                        '
                          ' c - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - d
            """

        And the ways
            | nodes | name  | oneway |
            | ab    | in    | no     |
            | gfb   | merge | yes    |
            | eb    | merge | yes    |
            | bcd   | merge | yes    |

        When I route I should get
            | waypoints | route          | intersections                                          |
            | a,d       | in,merge,merge | true:90;false:75 false:90 true:120 false:270;true:270  |

    @negative
    Scenario: Don't accept turn-restrictions
        Given the node map
            """
                          c - - - - - - - - - - - - - - - - - - - - - - - - - - - - - d
                       /                                                                  \
            a - - - b                                                                        g - - h
                       \                                                                  /
                          e - - - - - - - - - - - - - - - - - - - - - - - - - - - - - f
            """

        And the ways
            | nodes  | name | oneway |
            | ab     | road | yes    |
            | befgh  | road | yes    |
            | bcdg   | road | yes    |

        # This is an artificial scenario - not reasonable. It is only to test the merging on turn-restrictions
        And the relations
            | type        | way:from | way:to | node:via | restriction  |
            | restriction | ab       | bcdg   | b        | no_left_turn |

        When I route I should get
            | waypoints | route     | intersections                                                            |
            | a,h       | road,road | true:90,false:75 true:120 false:270,true:90 false:240 false:285;true:270 |

    Scenario: Actual Turn into segregated ways
        Given the node map
            """
            a - - - b - < - < - < - < - < - < - < - < - < - < - < c -
                    |                                                 \
                    |                                                 |
                    |                                                 |
                      d                                               |
                       \                                              |
                         \                                            |
                          e > - > - > - > - > - > - > - > - > - > - > f - - - - - - g
            """

        And the ways
            | nodes | name | oneway |
            | ab    | road | no     |
            | fcb   | road | yes    |
            | bdef  | road | yes    |
            | fg    | road | no     |

        When I route I should get
            | waypoints | route          | intersections                                                           |
            | a,g       | road,road,road | true:90,false:90 true:165 false:270,true:90 false:270 true:345;true:270 |

    Scenario: Merging parallel roads with intermediate bridges
    # https://www.mapillary.com/app/?lat=52.466483333333336&lng=13.431908333333332&z=17&focus=photo&pKey=LWXnKqoGqUNLnG0lofiO0Q
    # http://www.openstreetmap.org/#map=19/52.46750/13.43171
        Given the node map
            """
                f
                |
               .e.
              /   \
             /     \
            g       d
            |       |
            |       |
            |       |
            |       |
            |       |
            |       |
            |       |
            |       |
            h       c
             \     /
              \   /
               \ /
                b
                |
                a
                |
                |
            r - x - s
                |
                |
                y
            """

        And the ways
            | nodes | name           | highway   | oneway |
            | ab    | Hermannstr     | secondary |        |
            | bc    | Hermannstr     | secondary | yes    |
            | cd    | Hermannbruecke | secondary | yes    |
            | de    | Hermannstr     | secondary | yes    |
            | ef    | Hermannstr     | secondary |        |
            | eg    | Hermannstr     | secondary | yes    |
            | gh    | Hermannbruecke | secondary | yes    |
            | hb    | Hermannstr     | secondary | yes    |
            | xa    | Hermannstr     | secondary |        |
            | yx    | Hermannstr     | secondary |        |
            | rxs   | Silbersteinstr | tertiary  |        |

        And the nodes
            | node | highway         |
            | x    | traffic_signals |

        When I route I should get
            | waypoints | turns         | route                 | intersections   |
            | a,f       | depart,arrive | Hermannstr,Hermannstr | true:180;true:0 |
            | f,a       | depart,arrive | Hermannstr,Hermannstr | true:180;true:0 |
            | y,f       | depart,arrive | Hermannstr,Hermannstr | true:180;true:0 |
            | f,y       | depart,arrive | Hermannstr,Hermannstr | true:180;true:0 |

    Scenario: Four Way Intersection Double Through Street Segregated
        Given the node map
            """
                        q             p
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        b             c
                         \           /
                          \         /
                           \       /
            j - - i  .      \     /       , d - - o
                        .    \   /    .
                           .  \ /  .
                             > a <
                           .  / \  '
                        .    /   \    '
                     .      /     \       '
            k - - h        /       \        e - - n
                          /         \
                         /           \
                        g             f
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        |             |
                        l             m
            """

        And the ways
            | nodes  | highway | oneway | name   |
            | khaij  | primary | yes    | first  |
            | odaen  | primary | yes    | first  |
            | qbacp  | primary | yes    | second |
            | mfagl  | primary | yes    | second |

       When I route I should get
            | waypoints | route                | turns                        |
            | f,e       | second,first,first   | depart,turn right,arrive     |
            | f,c       | second,second        | depart,arrive                |
            | f,i       | second,first,first   | depart,turn left,arrive      |
            | f,g       | second,second,second | depart,continue uturn,arrive |
            | d,c       | first,second,second  | depart,turn right,arrive     |
            | d,i       | first,first          | depart,arrive                |
            | d,g       | first,second,second  | depart,turn left,arrive      |
            | d,e       | first,first,first    | depart,continue uturn,arrive |
            | b,i       | second,first,first   | depart,turn right,arrive     |
            | b,g       | second,second        | depart,arrive                |
            | b,e       | second,first,first   | depart,turn left,arrive      |
            | b,c       | second,second,second | depart,continue uturn,arrive |
            | h,g       | first,second,second  | depart,turn right,arrive     |
            | h,e       | first,first          | depart,arrive                |
            | h,c       | first,second,second  | depart,turn left,arrive      |
            | h,i       | first,first,first    | depart,continue uturn,arrive |
