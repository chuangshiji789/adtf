

//Lane keeping
const double lane_keeping_maxSpeed 	= 0.9;
const double lane_keeping_minSpeed 	= 0.4;
const double lane_keeping_halfSpeed = ((lane_keeping_maxSpeed + lane_keeping_minSpeed) * 0.5);

const double weightFact_Lanekeeping_HY	= 3;//3.2;//3
const double weightFact_Lanekeeping_LY	= 9;//6.5;//6.5
const double weightFact_Lanekeeping_DY	= 7;//7

const double weightFact_Lanekeeping_a	= (weightFact_Lanekeeping_HY - weightFact_Lanekeeping_LY)/(lane_keeping_maxSpeed - lane_keeping_minSpeed);
const double weightFact_Lanekeeping_b	= weightFact_Lanekeeping_LY - (weightFact_Lanekeeping_HY - weightFact_Lanekeeping_LY)/(lane_keeping_maxSpeed - lane_keeping_minSpeed) * lane_keeping_minSpeed;


//Turn left
const double weightFact_TurnLeft_X 	= 15;//15
const double weightFact_TurnLeft_Y	= 8;//8


//turn right
const double weightFact_TurnRight_X	= 15;
const double weightFact_TurnRight_Y	= 8;//8


//straight
const double weightFact_Straight_X	= 3;
const double weightFact_Straight_Y	= 10;


//pull out left
const double weightFact_PullOutLeft_X	= 15;
const double weightFact_PullOutLeft_Y	= 8;


//pull out right
const double weightFact_PullOutRight_X	= 20;
const double weightFact_PullOutRight_Y	= 8;


//parking
const double weightFact_Parking_X	= 10;
const double weightFact_Parking_Y	= 28;


//avoidance
const double weightFact_Avoidance_X	= 0;
const double weightFact_Avoidance_Y	= 15;


//merge left
const double weightFact_MergeLeft_X	= 8;
const double weightFact_MergeLeft_Y	= 15;


