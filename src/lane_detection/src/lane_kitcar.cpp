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
        Destination[0]=Point2f(160 * 0.5, 0);
        Destination[1]=Point2f(480 * 0.5, 0);
        Destination[2]=Point2f(160 * 0.5, 480 * 0.5);
        Destination[3]=Point2f(480 * 0.5, 480 * 0.5); 
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

  
        maxRadiusForCluster = 20;
        minPointsPerLandmark = 20;
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
            Mat copy_frame = frame.clone(); 
            Mat empty_frame = Mat::zeros( frame.rows, frame.cols, frame.type());
            //fillConvexPoly(copy_frame, Source_points, Scalar(0,0,0),8,0);
            Mat invertedPerspectiveMatrix;
            //draw_bird_eye_line(frame, Source, Destination); 
            Perspective(frame, invertedPerspectiveMatrix);
            img_edges = Threshold(frame);
            //img_edges.assignTo(img_edges, CV_8UC1); 
            points_.clear();
            clusteredPoints_.clear();
            cv::findNonZero(img_edges, points_);
            FindClusters(points_, clusteredPoints_, maxRadiusForCluster, minPointsPerLandmark, maxPointsPerLandmark);
            /*
            locate_lanes(img_edges, frame);
            draw_lines(frame);   
            warpPerspective(frame,empty_frame,invertedPerspectiveMatrix,Size(frame.cols,frame.rows));
            bitwise_xor(copy_frame , empty_frame, frame);
            ss.str(" ");
    	    ss.clear();
            ss<<"[ANG]: "<<angle;
            putText(frame, ss.str(), Point2f(2,20), 0,1, Scalar(0,255,255), 2);
            */
            //cout << "Cluster points"<< clusteredPoints_.size() <<endl;
            for (auto& cluster : clusteredPoints_)
                ReduceClusters(cluster);
            sort(clusteredPoints_.begin(), clusteredPoints_.end(), 
            [&](const Cluster& cluster1, const Cluster& cluster2){
                auto last_point1 = cluster1.end()-1;
                auto last_point2 = cluster2.end()-1;
                return (last_point1->y > last_point2->y);
            });
            //show_clusters(frame, clusteredPoints_);
            //print_clusters(clusteredPoints_);
            locate_histogram = Histogram(img_edges);
            Find_lane_points(clusteredPoints_);
            show_lane_points(frame, left_points);
            show_lane_points(frame, right_points);
            //circle(frame, cv::Point(locate_histogram[0],  frame.rows -1),cvRound((double)4/ 2), Scalar(255, 0, 0),2);
            //circle(frame, cv::Point(locate_histogram[1],  frame.rows -1),cvRound((double)4/ 2), Scalar(0, 255, 0),2);
            resizeWindow(OPENCV_WINDOW,frame.cols, frame.rows);
          	cv::imshow(OPENCV_WINDOW,frame);
            cv::waitKey(30);
            /*
            center_message.data = distance_center;
            center_pub.publish(center_message);
	    	angle_line_message.data = angle;
	    	angle_line_pub.publish(angle_line_message);
            */
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
            Sobel(frameGray, sobelx, CV_32F, 1, 0);
            minMaxLoc(sobelx, &minVal, &maxVal);
            sobelx.convertTo(frameGray, CV_8U, 255.0/(maxVal), - 255.0/(maxVal));
	        return frameGray;
        }

	    void resize_image(Mat &input,float alpha=0.15)
        {
		    cv::resize(input,input,Size(input.cols*alpha,input.rows*alpha));
	    }

        void FindClusters(const std::vector<cv::Point>& points_in,
                                  std::vector<Cluster>& clusters,
                                  const double radiusThreshold,
                                  const unsigned int minPointsThreshold,
                                  const unsigned int maxPointsThreshold) const {
            for (auto& thisPoint : points_in)  /// go thru all points
            {
                bool clusterFound = 0;  /// set flag that not used yet
                /// the last created cluster is most liley the one we are looking for
                for (auto& cluster : boost::adaptors::reverse(clusters)) {  /// go thru all clusters
                    for (auto& clusterPoint : cluster) {  /// go thru all points in this cluster
                        /// if distance is smaller than threshold, add point to cluster
                        if (cv::norm(clusterPoint - thisPoint) <= radiusThreshold) {
                            cluster.push_back(thisPoint);
                            clusterFound = true;
                            break;  /// because point has been added to cluster, no further search is neccessary
                        }
                    }
                    if (clusterFound)  /// because point has been added to cluster, no further search is neccessary
                        break;
                }

                if (!clusterFound)  /// not assigned to any cluster
                {
                    Cluster newCluster;               /// create new cluster
                    newCluster.push_back(thisPoint);  /// put this point in this new cluster
                    clusters.push_back(newCluster);   /// add this cluster to the list
                }
            }

            /// second rule: check for minimum and maximum of points per cluster
            clusters.erase(std::remove_if(clusters.begin(),
                                clusters.end(),
                                [&](Cluster& cluster) {
                                  return (minPointsThreshold > cluster.size() ||
                                          maxPointsThreshold < cluster.size());
                                          }),clusters.end());
        }

       void ReduceClusters(Cluster& cluster){
            map<int, vector<int>>  reduce_cluster;
            for (auto point : cluster){
                reduce_cluster[point.y].push_back(point.x);
            }       
            cluster.clear();
            for (auto point : reduce_cluster){
                int sum_of_elems{0}; 
                std::for_each(point.second.begin(), point.second.end(), 
                [&sum_of_elems] (int n) {
                    sum_of_elems += n;
                });
                cluster.push_back(Point(static_cast<int>(sum_of_elems/point.second.size()), point.first));
            }
        }

        void Find_lane_points(vector<Cluster> &clusteredPoints_){
            left_points.clear();
            right_points.clear();
            stack<Cluster> left_points_stack; 
            stack<Cluster> right_points_stack;
            // for left lane
            int index = 0;
            for (auto& cluster : clusteredPoints_){
                auto last = cluster.end()-1;
                if(abs(last->x - locate_histogram[0]) < 40){
                    left_points_stack.push(cluster);
                    clusteredPoints_.erase(clusteredPoints_.begin() + index);
                    break;
                }
                ++index;
            }
            if (!left_points_stack.empty()){
                for (vector<Cluster>::iterator cluster =  clusteredPoints_.begin(); cluster != clusteredPoints_.end(); ){
                    auto first = left_points_stack.top().begin();
                    auto last =  cluster->end()-1;
                    if (cv::norm(*first - *last) <= 75) {
                        left_points_stack.push(*cluster);
                        cluster = clusteredPoints_.erase(cluster);
                    }
                    else{
                        ++cluster;
                    }
                }
                while (!left_points_stack.empty()) {
                    for (auto& point : left_points_stack.top()){
                        left_points.push_back(point);
                    }
                    left_points_stack.pop();
                }
            }

            index = 0;
            for (auto& cluster : clusteredPoints_){
                auto last = cluster.end()-1;
                if(abs(last->x - locate_histogram[1]) < 40){
                    right_points_stack.push(cluster);
                    clusteredPoints_.erase(clusteredPoints_.begin() + index);
                    break;
                }
                ++index;
            }

            if (!right_points_stack.empty()){
                for (vector<Cluster>::iterator cluster =  clusteredPoints_.begin(); cluster != clusteredPoints_.end(); ){
                    auto first = right_points_stack.top().begin();
                    auto last =  cluster->end()-1;
                    if (cv::norm(*first - *last) <= 75) {
                        right_points_stack.push(*cluster);
                        cluster = clusteredPoints_.erase(cluster);
                    }
                    else{
                        ++cluster;
                    }
                }
                while (!right_points_stack.empty()) {
                    for (auto& point : right_points_stack.top()){
                        right_points.push_back(point);
                    }
                    right_points_stack.pop();
                }
            }


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

		void width_filter(Mat &img,int width_max=20)
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
	        	for(int c = 0;c<img.cols;c++)
                {
                    now_point = *(pixel+c);
                    if(last_point == 0 && now_point > 0)
                        start = c;
                    if(start != -1)
                        count_white++;
                    if(last_point > 0 && now_point == 0)
                    {
                        if(count_white >= width_max)
                        {
                            for(int k = start;k < c;k++)
                                *(pixel+k) = 0;
                        }
                        start = -1;
                        count_white = 0;
                    }
                    last_point = now_point;
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

};
int main(int argc,char **argv){
    ros::init(argc, argv, "line_detection");
    LineDetection *ic= new  LineDetection;
    ros::spin();
    return 0;
}