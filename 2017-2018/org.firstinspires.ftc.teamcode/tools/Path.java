package org.firstinspires.ftc.teamcode.tools;

import java.util.ArrayList;

/**
 * Created by Jonathan on 11/28/2017.
 */

public class Path {
    public static  Point[] STRAIGHT_PATH = {
        new Point( 570 , 570 ),
        new Point(0,0)
    };
    public static final Point[] POSITION_ONE_PATH = {
        new Point( 1761, 1740 ),
        new Point( 739, -717 ),
        new Point( 1311, 1297 ),
        new Point( 704, -725 ),
        new Point( 925, 907 ),
        new Point( -750, 725 ),
        new Point( -100, -100 ),
        /*new Point( 1669 , 1637 ),
        new Point( 789 , -798 ),
        new Point( 1341 , 1451 ),
        new Point( 725 , -736 ),
        new Point( 714 , 707 ),
        new Point( -765 , 756 ),
        new Point( -200, -200)*/
        /*new Point( 1160, 1157 ),
        new Point( 812 , -669 ),
        new Point( 1340 , 1223 ),
        new Point( 776 , -551 ),
        new Point( 341 , 419 ),
        new Point( -693 , 645 ),
        new Point( -200, -200 ),*/
    };
    public static final Point[] POSITION_TWO_PATH = {
        new Point( 1538 , 1380 ),
        new Point( 911 , -930 ),
        new Point( 1944 , 1960 ),
        new Point( 454 , -487 )
    };
    public static final Point[] POSITION_FOUR_PATH = {
        new Point( 1637, 1669 ),
        new Point( -798, 789 ),
        new Point( 1451, 1341 ),
        new Point( -736, 725 ),
        new Point( 907, 914 ),
        new Point( 756, -765 ),
        new Point( -100, -100)
    };
    public static final Point[] POSITION_THREE_PATH = {
        new Point( 1652, 1626),
        new Point( -946, 941),
        new Point(2270, 2229),
        new Point(-700, 655),
        new Point(759, 616),
        // new Point( 1380 , 1538),
        // new Point( -930 , 911 ),
        // new Point( 1960 , 1944 ),
        // new Point( -487 , 454 )
    };
    public static  Point[] LEFT_TURN = {
        new Point( -512, 495),
        new Point( 342, 317),
        new Point( 334, -442),
        new Point( 259, 260),
        /*new Point( -314 , 269 ),
        new Point ( 724 , 712 ),
        new Point( 242 , -257 ),
        new Point( 356 , 372 ),*/
        new Point(0, 0)
    };
    public static  Point[] RIGHT_TURN = {
        new Point( 495, -512),
        new Point( 317, 342),
        new Point( -442, 334),
        new Point( 260, 259),
        /*new Point( 291 , -249 ),
        new Point( 574 , 560 ),
        new Point( -319 , 263 ),
        new Point( 485 , 456 ),*/
        new Point(0, 0)
    };
    private int index = 0;
    private ArrayList<Point> path;
    public Path() {
        path = new ArrayList<Point>();
    }
    public Path(Point[] p) {
        path = new ArrayList<Point>();
        for (Point po : p) {
            path.add(po);
        }
    }
    public void addPoint(Point p) {
        path.add(p);
    }
    public void addList(Point[] p) {
        for (Point po : p) {
            path.add(po);
        }
    }
    public Point peek() {
        Point p = path.get(index);
        return p;
    }
    public void inc() {
        index++;
    }
    public Point prev() {
        if (index-1 >= 0) {
            Point p = path.get(index - 1);
            return p;
        } else {
            return new Point(0,0);
        }
    }
    public Point getLastPoint() {
        return path.get(path.size()-1);
    }
    public int getIndex() {
        return index;
    }
    public boolean isComplete() {
        return (index == path.size()-1);
    }
}
