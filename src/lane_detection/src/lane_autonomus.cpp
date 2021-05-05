#include<iostream>
#include<math.h>
#include<string>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
/*Using image transport for publish and subscribing to image in ros,
allows subscriber to compressed image stream*/
#include <image_transport/image_transport.h>
/*Using the headers for CvBridge, to image encondings*/
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
/*Include the headers for opencv image processing and GUI modules */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//Library to use time of computer
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <map>
#include <stack>
#include <queue>
#include <boost/range/adaptor/reversed.hpp>
using namespace cv;
using namespace std;

typedef std::vector<cv::Point> Cluster;

//static const std::string OPENCV_WINDOW = "Original";
static const std::string OPENCV_WINDOW = "LANE DETECTION";
class LineDetection{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher center_pub;
		ros::Publisher angle_line_pub;
        std_msgs::Int16 center_message;
		std_msgs::UInt8 angle_line_message;
        std::chrono::time_point<std::chrono::system_clock> start,end;
        std::chrono::duration<double> elapsed_seconds;
        stringstream ss;
        /*variable image*/
        Mat frame;
        Mat img_edges;
        Mat img_lines;
        Mat img_perspective;
		vector<Point> left_points,right_points; 
        vector<Point> left_line,right_line,center_line;
        float polyleft[3],polyright[3];
 	    float polyleft_last[3],polyright_last[3];
        Point2f Source[4]; 
        Point2f Destination[4];
        //vector<Point> Source_points;
		int center_cam;
    	int center_lines;
		int distance_center;
		int angle;
        std::vector<Cluster> clusteredPoints_;
        std::vector<cv::Point> points_;
        double maxRadiusForCluster; /**< Maximum radius for clustering marker points to landmarks*/
        uint16_t minPointsPerLandmark; /**< Minimum count of marker points per landmark (0)*/
        uint16_t maxPointsPerLandmark; 
        vector<int> locate_histogram;
         // parameters for clustering

    public:
        //Constructor
        LineDetection():it_(nh_){
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/app/camera/rgb/image_raw",1,&LineDetection::LineDetectionCb, this);
        center_pub = nh_.advertise<std_msgs::Int16>("/distance_center_line",1000);
	    angle_line_pub = nh_.advertise<std_msgs::UInt8>("/angle_line_now",1000);
        cv::namedWindow(OPENCV_WINDOW, WINDOW_KEEPRATIO);
 
        Source[0] = Point2f(160 * 0.5, 300 * 0.5);
        Source[1] = Point2f(480 * 0.5, 300 * 0.5);
        Source[2] = Point2f(0, 400 * 0.5);
        Source[3] = Point2f(640 * 0.5, 400 * 0.5);
        //init points to desinations
        Destination[0]=Point2f(180 * 0.5, 0);
        Destination[1]=Point2f(460 * 0.5, 0);
        Destination[2]=Point2f(180 * 0.5, 480 * 0.5);
        Destination[3]=Point2f(460 * 0.5, 480 * 0.5); 
        /*
        Source_points.push_back(Point(500* 0.5, 280* 0.5));
        Source_points.push_back(Point(640* 0.5, 340* 0.5));
        Source_points.push_back(Point(0, 340* 0.5));
        Source_points.push_back(Point(140 * 0.5, 280* 0.5));
        */
	   	distance_center = 0.0;
	    center_cam = 0;
        center_lines = 0;
		angle = 0;

  
        maxRadiusForCluster = 15;
        minPointsPerLandmark = 75;
        maxPointsPerLandmark = 1000;
        }
        //Destructor
        ~LineDetection(){
          	cv::destroyWindow(OPENCV_WINDOW);
        }
        //Callback funtion
        void LineDetectionCb(const sensor_msgs::ImageConstPtr& msg)
        {
            set_start_time();
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                copyImageRGB(cv_ptr->image,frame);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
	       	resize_image(frame,0.5);
            Mat copy_frame;
            Mat empty_frame = Mat::zeros( frame.rows, frame.cols, frame.type());
            //fillConvexPoly(copy_frame, Source_points, Scalar(0,0,0),8,0);
            Mat invertedPerspectiveMatrix;
            //draw_bird_eye_line(frame, Source, Destination); 
            Perspective(frame, invertedPerspectiveMatrix);
            img_edges = Threshold(frame);
            find_points(img_edges, left_points, right_points,locate_histogram[0], locate_histogram[1]);
            copy_frame = frame.clone();
            show_lane_points(copy_frame, left_points);
            show_lane_points(copy_frame, right_points);
            //circle(frame, cv::Point(locate_histogram[0],  frame.rows -1),cvRound((double)4/ 2), Scalar(255, 0, 0),2);
            //circle(frame, cv::Point(locate_histogram[1],  frame.rows -1),cvRound((double)4/ 2), Scalar(0, 255, 0),2);
            cout << "width line" << abs(locate_histogram[0]-locate_histogram[1]);
            
            draw_lines(frame);   
            /*
            warpPerspective(frame,empty_frame,invertedPerspectiveMatrix,Size(frame.cols,frame.rows));
            bitwise_xor(copy_frame , empty_frame, frame);
            ss.str(" ");
    	    ss.clear();
            ss<<"[ANG]: "<<angle;
            putText(frame, ss.str(), Point2f(2,20), 0,1, Scalar(0,255,255), 2);
            */
            resizeWindow(OPENCV_WINDOW,frame.cols, frame.rows);
          	cv::imshow(OPENCV_WINDOW,copy_frame);
            cv::waitKey(15);
            center_message.data = distance_center;
            center_pub.publish(center_message);
	    	angle_line_message.data = angle;
	    	angle_line_pub.publish(angle_line_message);
            set_end_time();
            ROS_INFO("[FPS]: %i ",FPS_subscriber());
        }

        void set_start_time()
        {
            start = std::chrono::system_clock::now();
        }
        void set_end_time()
        {
            end = std::chrono::system_clock::now();
        }
        int FPS_subscriber()
        {
            elapsed_seconds = end-start;
            float t = elapsed_seconds.count();
            return 1/t;
        }
        //Method to change image from BGR to RGB
        void copyImageRGB(Mat & image_BGR,Mat &image_RGB)
        {
            cvtColor(image_BGR, image_RGB, COLOR_BGR2RGB);
        }
        //Method to rotate image 
        void rotate(Mat &src, double angle=180.0)
        {
            Point2f pt(src.cols/2., src.rows/2.);    
            Mat r = getRotationMatrix2D(pt, angle, 1.0);
            warpAffine(src, src, r, Size(src.cols, src.rows));
        }
        void Perspective(Mat &frame,Mat &invertedPerspectiveMatrix)
        {
 	        Mat matrixPerspective( 2, 4, CV_32FC1 );
	        matrixPerspective = Mat::zeros( frame.rows, frame.cols, frame.type() );
            matrixPerspective = getPerspectiveTransform(Source,Destination); 
			warpPerspective(frame,frame,matrixPerspective,Size(frame.cols,frame.rows));
            invert(matrixPerspective, invertedPerspectiveMatrix);
        }
        
        void draw_bird_eye_line(Mat &frame,Point2f *Source,Point2f *Destination)
        {
	        line(frame,Source[0],Source[1],Scalar(0,0,255),2);
            line(frame,Source[1],Source[3],Scalar(0,0,255),2);
            line(frame,Source[2],Source[3],Scalar(0,0,255),2);
            line(frame,Source[2],Source[0],Scalar(0,0,255),2);
            line(frame,Destination[0],Destination[1],Scalar(0,255,0),2);
            line(frame,Destination[1],Destination[3],Scalar(0,255,0),2);
            line(frame,Destination[2],Destination[3],Scalar(0,255,0),2);
            line(frame,Destination[2],Destination[0],Scalar(0,255,0),2);
        }

        Mat Threshold(Mat frame)
        {
	        Mat frameGray;
            Mat sobelx;    
            Mat draw;
            double minVal, maxVal;
	        cvtColor(frame, frameGray, COLOR_BGR2GRAY);
            medianBlur(frameGray, frameGray, 5);
	        inRange(frameGray, 120, 255, frameGray);
            locate_histogram = Histogram(frameGray);
             white_filter(frameGray, 16);
	        return frameGray;
        }

	    void resize_image(Mat &input,float alpha=0.15)
        {
		    cv::resize(input,input,Size(input.cols*alpha,input.rows*alpha));
	    }

        void find_points(Mat &frame, std::vector<cv::Point>& points_find_left, std::vector<cv::Point>& points_find_right, int point_x_start_left, int point_x_start_right){
            points_find_left.clear();
            points_find_right.clear();
            int frame_height = frame.rows;
            int frame_width = frame.cols;
            int index_left;
            int index_right;
            int current_row;
            int const ROW_STEP = 1;
            int const margin_x = 25;
            int  margin_back_x_right, margin_top_x_right, margin_back_x_left, margin_top_x_left;
            std::vector<Point> image_row_vector_left, image_row_vector_right;
            Point near_point_left, near_point_right;
            index_left = point_x_start_left;
            index_right = point_x_start_right;
            current_row = frame_height - ROW_STEP;
            /*
             if (abs(point_x_start_left - point_x_start_right) < 100)
                return;
             */   
            points_find_left.push_back(cv::Point(index_left, current_row));
            points_find_right.push_back(cv::Point(index_right, current_row));

            while (current_row > 1){
                /* scanning */
                margin_back_x_left = index_left - margin_x  < 0 ? 0 : index_left - margin_x;
                margin_top_x_left = index_left + margin_x  > frame_width-1 ? frame_width-1 : index_left + margin_x;


                image_row_vector_left = find_row_point(frame, current_row, margin_back_x_left, margin_top_x_left);
                
                if (image_row_vector_left.size() > 0){
                    near_point_left = find_near_point(image_row_vector_left, points_find_left.back());
                    if (cv::norm(near_point_left -  points_find_left.back() ) < 60){
                        points_find_left.push_back(near_point_left);
                        index_left = near_point_left.x;
                    }
                }
                
                margin_back_x_right = index_right - margin_x  < 0 ? 0 : index_right - margin_x;
                margin_top_x_right = index_right + margin_x  > frame_width-1 ? frame_width-1 : index_right + margin_x;


                image_row_vector_right = find_row_point(frame, current_row, margin_back_x_right, margin_top_x_right);
                
                if (image_row_vector_right.size() > 0){
                    near_point_right = find_near_point(image_row_vector_right, points_find_right.back());
                    if (cv::norm(near_point_right -  points_find_right.back() ) < 60){
                        points_find_right.push_back(near_point_right);
                        index_right = near_point_right.x;
                    }
                }
                
                current_row -= ROW_STEP;
            }
            if (left_points.size() > right_points.size()){
                if (abs(left_points[0].x - right_points[0].x) < 75){
                    right_points.clear();
                    left_points.clear();
                }
            }
            else{
                 if (abs(right_points[0].x - left_points[0].x) < 75){
                    left_points.clear();
                    right_points.clear();
                }
            }
        }


        vector<Point> find_row_point(Mat frame, int current_row, int margin_back_x, int margin_top_x){
            vector<Point> NonBlack;
                uchar now_point;
			    uchar *pixel = frame.ptr<uchar>(current_row);
	        	for(int c = margin_back_x; c < margin_top_x; ++c)
                {
                    if(now_point = *(pixel+c) > 0)
                        NonBlack.push_back(Point(c, current_row));
                }
                return NonBlack;
        }

        Point find_near_point(const vector<cv::Point>& points_find, Point last_point){
            
            map<int,Point> points_dist;
            int dist;
            for(auto point : points_find){
                dist = static_cast<int>(cv::norm(point - last_point));
                points_dist[dist] = point; 
            }
            return points_dist.begin()->second;
        }

        void show_lane_points(Mat &frame,const Cluster& cluster){
                int colors[3];
                colors[0] = static_cast<int>(rand() % 255);
                colors[1] = static_cast<int>(rand() % 255);
                colors[2] = static_cast<int>(rand() % 255);
                for (auto point : cluster){
                    circle(frame, cv::Point(point.x , point.y),cvRound((double)4/ 2), Scalar(colors[0], colors[1], colors[2]),2);
                }
        }

        void show_clusters(Mat &frame,const vector<Cluster> &clusteredPoints_){
            int colors[3];
            for (auto cluster : clusteredPoints_){
                colors[0] = static_cast<int>(rand() % 255);
                colors[1] = static_cast<int>(rand() % 255);
                colors[2] = static_cast<int>(rand() % 255);
                for (auto point : cluster){
                    circle(frame, cv::Point(point.x , point.y),cvRound((double)4/ 2), Scalar(colors[0], colors[1], colors[2]),2);
                }
            }
        }

        void print_clusters(const vector<Cluster> &clusteredPoints_){
            int i = 0;
            for (auto cluster : clusteredPoints_){
                cout << endl << endl;
                cout << "Cluster "<< " --- " << ++i << endl << endl;
                for (auto point : cluster){
                    cout << "X: " << point.x << " --- " << "Y: " << point.y << endl;
                }
            }
        } 

         void white_filter(Mat &img,int width_max=20)
        {
			uchar last_point;
			int count_white;
            int start;
			uchar now_point;
			for(int r = 0;r < img.rows;r++)
            {
			    uchar *pixel = img.ptr<uchar>(r);
                last_point = 0;
                start  = -1;
                count_white = 0;
                int c = 0;
	        	while(c<img.cols)
                {
                    now_point = *(pixel + c);
                    if(last_point == 0 && now_point > 0)
                        start = c;
                    if(start != -1)
                        count_white++;
                    if(last_point > 0 && now_point == 0 && start != -1 )
                    {
                        if(count_white >= width_max)
                        {
                            for(int k = start; k < c; k++)
                                *(pixel+k) = 0;
                        }
                        start = -1;
                        count_white = 0;
                    }
                    last_point = now_point;
                    ++c;
                }
            }    
		}

	    vector<int> Histogram(Mat &img)
        {
            //Create histogram with the length of the width of the frame 
	        vector<int> histogramLane;
	        vector<int> LanePosition(2);
			int init_row,end_row;
	        Mat ROILane;
	        Mat frame;  
		    init_row = img.rows*2/3;	
		    end_row = img.rows/3-1;
	    	img.copyTo(frame);
            for(int i=0;i<img.cols;i++)
            {
                //Region interest
                ROILane=frame(Rect(i,init_row ,1,end_row));
                //Normal values 
                divide(255,ROILane,ROILane);
                //add the value 
                histogramLane.push_back((int)(sum(ROILane)[0]));
            } 
            //Find line left
            vector<int>:: iterator LeftPtr;
            LeftPtr = max_element(histogramLane.begin(),histogramLane.begin()+img.cols/2);
            LanePosition[0] = distance(histogramLane.begin(),LeftPtr);
            //find line right
            vector<int>:: iterator RightPtr;
            RightPtr = max_element(histogramLane.begin()+(img.cols/2)+1,histogramLane.end());
            LanePosition[1] = distance(histogramLane.begin(),RightPtr);
	        return  LanePosition;
        }

	bool regression_left(){

        if (left_points.size() <= 0)
            return false;

	    long sumX[5] = {0,0,0,0,0};
	    long sumY[3] = {0,0,0};
	    long pow2 = 0;

	    for (auto point = left_points.begin(); point != left_points.end(); point++){
		    pow2 = (point->y) * (point->y);
	        sumX[0]++;
		    sumX[1] += (point->y);
		    sumX[2] += pow2;
		    sumX[3] += pow2 * (point->y);
		    sumX[4] += pow2 * pow2;
		    sumY[0] += (point->x);
		    sumY[1] += (point->x) * (point->y);
		    sumY[2] += (point->x) * pow2;
	    }	
        solve_system(sumX, sumY, polyleft);
        return true;
	}

    bool regression_right(){

        if (right_points.size() <= 0)
            return false;
	    long sumX[5] = {0,0,0,0,0};
	    long sumY[3] = {0,0,0};
	    long pow2 = 0;
	    
        for (auto point = right_points.begin(); point != right_points.end(); point++){
		    pow2 = (point->y) * (point->y);
	        sumX[0] ++;
		    sumX[1] += (point->y);
		    sumX[2] += pow2;
		    sumX[3] += pow2 * (point->y);
		    sumX[4] += pow2 * pow2;
		    sumY[0] += (point->x);
		    sumY[1] += (point->x) * (point->y);
		    sumY[2] += (point->x) * pow2;
	    }	
        solve_system(sumX, sumY, polyright);
        return true;
	}
 	void solve_system(long *sX,long *sY,float *x){
	    int n,i,j,k;
        n=3;
    	float a[n][n+1];
        //declare an array to store the elements of augmented-matrix    
    	//"Enter the elements of the augmented-matrix row-wise
        a[0][0] = sX[0];   
        a[0][1] = sX[1];   
        a[0][2] = sX[2];   
        a[0][3] = sY[0];
        ////////////////   
        a[1][0] = sX[1];   
        a[1][1] = sX[2];   
        a[1][2] = sX[3];   
        a[1][3] = sY[1];  
        ////////////////   
        a[2][0] = sX[2];   
        a[2][1] = sX[3];   
        a[2][2] = sX[4];   
        a[2][3] = sY[2];   
        ////////////////
        for (i = 0; i < n; i++)                    //Pivotisation
            for (k = i+1; k<n; k++)
                if (abs(a[i][i]) < abs(a[k][i]))
                    for (j = 0; j <= n; j++){
                        double temp = a[i][j];
                        a[i][j] = a[k][j];
                        a[k][j] = temp;
                    }
        for (i = 0; i <n-1; i++)            //loop to perform the gauss elimination
            for (k = i+1; k < n; k++){
                double t = a[k][i] / a[i][i];
                for (j = 0; j <= n; j++)
                    a[k][j] = a[k][j] - t * a[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
        for (i = n-1; i >= 0; i--)                //back-substitution
        {                        //x is an array whose values correspond to the values of x,y,z..
            x[i] = a[i][n];                //make the variable to be calculated equal to the rhs of the last equation
            for (j = i+1; j<n; j++)
                if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                    x[i] = x[i] - a[i][j] * x[j];
            x[i] = x[i] / a[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
        } 
	}

    void draw_lines(Mat &img)
    {
	    float columnL,columnL_aux;
		float columnR,columnR_aux;
	    float row;
	    float m = 0.0,b = 0.0,c = 0.0;
		bool find_line_left;
		bool find_line_right;
		float angle_to_mid_radian;
		find_line_right = regression_right();
		find_line_left = regression_left();
		right_points.clear();
	    left_points.clear();
		center_cam = (img.cols / 2) - 10;
        if(find_line_left && find_line_right){
            
            for(row = img.rows-1;row >= 0; row -= 8){
                columnR = polyright[0] + polyright[1] * (row) + polyright[2] * (row * row);
                circle(img, cv::Point(columnR, row), cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
                columnL = polyleft[0] + polyleft[1] * (row)+polyleft[2] * (row * row);
                circle(img, cv::Point( columnL, row), cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
            }

            center_lines = (columnR + columnL) / 2;
            distance_center = center_cam - center_lines;
		    
            if(distance_center == 0)
			    angle = 90;
		    else{
			    angle_to_mid_radian = atan(static_cast<float>(0 - (img.rows - 1)) / static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                
                if(angle < 0 && angle > (0 - 90))
                    angle = (0-1) * (angle);
                
                else if(angle > 0 && angle < 90)
                    angle = 180 - angle; 
		    }
            
            line(img, Point(center_lines, 0), Point(center_cam, (img.rows - 1)), Scalar(0, 0, 255), 2); 
            
            for(int k = 0; k < 3;k++)
            {
                polyleft_last[k] = polyleft[k]; 
                polyright_last[k] =polyright[k];
            }
        }
        else if(find_line_left){
            
            for(row = img.rows-1; row >=0 ; row -= 8){
                columnL = polyleft[0] + polyleft[1]*(row)+polyleft[2]*(row*row);
                circle(img, cv::Point(columnL, row), cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
            }
            //columnL = polyleft[0] + polyleft[1]*(0.0)+polyleft[2]*(0.0);
            columnL = polyleft[0];
            columnL_aux =  polyleft[0] + polyleft[1] * static_cast<float>(img.rows - 1) + polyleft[2] * ((img.rows - 1) * (img.rows - 1));
            
            if(columnL_aux == columnL){
                angle = 90;
                center_lines = columnL_aux;
            }
		    else{
			    angle_to_mid_radian = atan(static_cast<float>(0 - (img.rows - 1)) / static_cast<float>(columnL - columnL_aux));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                
                if(angle < 0 && angle > (0 - 90))
                    angle = (0 - 1) * (angle);
                
                else if(angle > 0 && angle < 90)
                    angle = 180 - angle; 
                
                if(angle < 90){
                    angle_to_mid_radian =  (float)(angle) * 0.0174533;
                    center_lines = center_cam + (int)(360.0 * cos(angle_to_mid_radian));
                }
                else{
                    angle_to_mid_radian =  (float)(180 - angle) * 0.0174533;
                    center_lines = center_cam - (int)(360.0 * cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
            
            for(int k = 0; k < 3;k++)
                polyleft_last[k] = polyleft[k]; 
        }
       
        else if(find_line_right){
            
            for(row = img.rows - 1; row >= 0; row -= 8){
                columnR = polyright[0] + polyright[1] * (row) + polyright[2] * (row * row);
                circle(img,cv::Point(columnR,row),cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
            }
            //columnR = polyright[0] + polyright[1]*(0.0)+polyright[2]*(0.0);
            columnR = polyright[0];
            columnR_aux = polyright[0] + polyright[1] * static_cast<float>(img.rows - 1) + polyright[2] * static_cast<float>((img.rows - 1) * (img.rows - 1));
            
            if(columnR_aux == columnR){
                angle = 90;
                center_lines = columnR_aux;
            }
		    else{
			    angle_to_mid_radian = atan(static_cast<float>(0 - (img.rows - 1)) / static_cast<float>(columnR - columnR_aux ));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                
                if(angle < 0 && angle > (0 - 90))
                    angle = (0 - 1) * (angle);
                
                else if(angle > 0 && angle < 90)
                    angle = 180 - angle; 
                
                if(angle < 90){
                    angle_to_mid_radian =  angle * 0.0174533;
                    center_lines = center_cam + (int)(360.0 * cos(angle_to_mid_radian));
                }
                else{
                    angle_to_mid_radian =  (float)(180 - angle) * 0.0174533;
                    center_lines = center_cam - (int)(360.0 * cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
             
            for(int k = 0;k < 3;k++)
                polyright[k] = polyright_last[k];
        }
        if(!find_line_left && !find_line_right){
            
            for(row = img.rows - 1; row >= 0; row -= 8){
                columnR = polyright_last[0] + polyright_last[1] * (row) + polyright_last[2] * (row * row);
                circle(img, cv::Point(columnR, row), cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
                columnL = polyleft_last[0] + polyleft_last[1] * (row) + polyleft_last[2] * (row * row);
                circle(img, cv::Point(columnL, row),cvRound((double)4 / 2), Scalar(255, 0, 0), 2);
            }
            center_lines = (columnR + columnL) / 2;
            distance_center = center_cam - center_lines;
		    
            if(distance_center == 0)
			    angle = 90;
		    else{
			    angle_to_mid_radian = atan(static_cast<float>(0 - (img.rows - 1)) / static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                
                if(angle < 0 && angle > (0 - 90))
                    angle = (0 - 1) * (angle);
                
                else if(angle > 0 && angle < 90)
                    angle = 180 - angle; 
		    }
        }
       // line(img, Point(center_cam,(img.rows / 4)), Point(center_cam,(img.rows * 3 / 4)), Scalar(0, 255, 0), 2); 
        line(img, Point(center_lines, 0), Point(center_cam, (img.rows - 1)), Scalar(0, 0, 255), 2);       
        }
};
int main(int argc,char **argv){
    ros::init(argc, argv, "line_detection");
    LineDetection *ic= new  LineDetection;
    ros::spin();
    return 0;
}