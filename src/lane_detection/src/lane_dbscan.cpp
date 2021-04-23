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
#include <chrono>
#include <ctime>
using namespace cv;
using namespace std;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

class PointC{
public:
    double x, y;
    int ptsCnt, cluster;
    double getDis(const PointC & ot) {
        return (x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y);
	//return  (x-ot.x) > 0 ? (x-ot.x) : (0-1)*(x-ot.x);
    }
};

class DBCAN {
public:
    int n, minPts;
    double eps; 
    vector<PointC> points;
    int size;
    vector<vector<int> > adjPoints;
    vector<bool> visited;
    vector<vector<PointC>> cluster;
    int clusterIdx;
    
    DBCAN(int n, double eps, int minPts, vector<PointC> points) {
        this->n = n;
        this->eps = eps* eps;
	//this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }
    void run () {
        checkNearPoints();
        
        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;
            
            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }
        
        cluster.resize(clusterIdx+1);
        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(points[i]);
            }
        }
	    for(int i=0;i<cluster.size();++i)
		reduce_vector(cluster[i]);

            for(int i=0;i<cluster.size();++i)
            {
                //minimun points by clauster
		        if(cluster[i].size()< n)
		            cluster.erase(cluster.begin()+(i--));
	    	}
    }
    
    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;
        
        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }
    
    void checkNearPoints() {
        for(int i=0;i<size;i++) {
            for(int j=0;j<size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        return points[idx].ptsCnt >= minPts;
    }
    
    vector<vector<PointC>> getCluster() {
        return cluster;
    }
	void  reduce_vector(vector<PointC> &data)
        {
            for(int i=0;i<data.size()-1;++i)
            {
                while(data[i].y==data[i+1].y)
		        {
		        if(i<data.size()-1)
		            data.erase(data.begin()+i);
		        else
		            break;
		        }
            }
        }
};


//static const std::string OPENCV_WINDOW = "Original";
static const std::string OPENCV_WINDOW = "LINES DETECTION";
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
		int center_cam;
    	int center_lines;
		int distance_center;
		int angle;
    /*
    int iLowH = 0;
    int iHighH = 179;
    int iLowS = 0;
    int iHighS = 255;
    int iLowV = 140;
    int iHighV = 215;
    */
    public:
        //Constructor
        LineDetection():it_(nh_)
        {
            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("/app/camera/rgb/image_raw",1,&LineDetection::LineDetectionCb, this);
            //center_pub = nh_.advertise<std_msgs::Int16>("/distance_center_line",1000);
	        //angle_line_pub = nh_.advertise<std_msgs::UInt8>("/angle_line_now",1000);
            cv::namedWindow(OPENCV_WINDOW, WINDOW_KEEPRATIO);
 
            /*
            createTrackbar("LowH", OPENCV_WINDOW, &iLowH,179);
            createTrackbar("HighH", OPENCV_WINDOW, &iHighH,179);
            createTrackbar("LowS", OPENCV_WINDOW, &iLowS,255);
            createTrackbar("HighS", OPENCV_WINDOW, &iHighS,255);
            createTrackbar("LowV", OPENCV_WINDOW, &iLowV,255);
            createTrackbar("HighV", OPENCV_WINDOW, &iHighV,255);
            */
            Source[0] = Point2f(140 * 0.5, 280* 0.5);
            Source[1] = Point2f(500* 0.5, 280* 0.5);
            Source[2] = Point2f(0, 340* 0.5);
            Source[3] = Point2f(640* 0.5, 340* 0.5);
            //init points to desination
            Destination[0]=Point2f(80* 0.5, 0);
            Destination[1]=Point2f(560* 0.5, 0);
            Destination[2]=Point2f(80* 0.5, 340* 0.5);
            Destination[3]=Point2f(560* 0.5, 340* 0.5); 
	   	    distance_center = 0.0;
	        center_cam = 0;
            center_lines = 0;
		    angle = 0;
        }
        //Destructor
        ~LineDetection()
        {
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
	        Perspective(frame);
            //draw_bird_eye_line(frame, Source, Destination);
		    //frame = frame(Rect(70,1,frame.cols-140,frame.rows-1));
	        img_edges = Threshold(frame);
	        locate_lanes(img_edges,frame);
            draw_lines(frame); 
            std::cout<<"Cols "<<frame.cols<<endl;
            std::cout<<"Rows "<<frame.rows<<endl;
            resizeWindow(OPENCV_WINDOW, frame.cols, frame.rows);
          	cv::imshow(OPENCV_WINDOW, frame);
            cv::waitKey(5);

          //center_message.data = distance_center;
           // center_pub.publish(center_message);
	    	//angle_line_message.data = angle;
	    	//angle_line_pub.publish(angle_line_message);
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

        void Perspective(Mat &frame)
        {
 	        Mat matrixPerspective( 2, 4, CV_32FC1 );
	        matrixPerspective = Mat::zeros( frame.rows, frame.cols, frame.type() );
            matrixPerspective = getPerspectiveTransform(Source,Destination); 
			warpPerspective(frame,frame,matrixPerspective,Size(frame.cols,frame.rows));
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
	        //width_filter(frameThreshold,20);
            Sobel(frameGray, sobelx, CV_32F, 1, 0);
            minMaxLoc(sobelx, &minVal, &maxVal);
            sobelx.convertTo(frameGray, CV_8U, 255.0/(maxVal), - 255.0/(maxVal));
	        return frameGray;
        }
	    void resize_image(Mat &input,float alpha=0.15)
        {
		    cv::resize(input,input,Size(input.cols*alpha,input.rows*alpha));
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
		            if(c == img.cols-1)
			            now_point = 0;
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
        vector<PointC> non_zeros(Mat &img)
        {
			uchar now_point;
            vector<PointC> points;
			for(int r = 0;r < img.rows;r++)
            {
			    uchar *pixel = img.ptr<uchar>(r);
	        	for(int c = 0;c<img.cols;c++)
                {
                    now_point = *(pixel+c);
		            if(now_point > 0)
                        points.push_back({(float)(c),(float)(r),0,NOT_CLASSIFIED}); 
		        }
            }    
            return points;
        } 

        vector<int> row_col(vector<PointC> &data,int n_rows)
        {
            vector<int> rows(n_rows,-1);
            for(auto i:data)
                rows[i.y] = i.x;
            return rows;
        }

        int dist_x(int x1,int x2)
        {
            int dist = (x1-x2); 
            return dist > 0 ? dist : (0-1)*(dist);
        }
		void locate_lanes(Mat &img,Mat &out_img)
        {
            left_points.clear();
            right_points.clear();
            vector<PointC> data;
	    vector<vector<PointC>> data_cluster;
            int MINIMUM_POINTS = 4.0;// minimum number of cluster
            int EPSILON = 60.0;// distance for clustering
            data = non_zeros(img);
            DBCAN dbScan(20,EPSILON,MINIMUM_POINTS,data);
            dbScan.run();
	    data_cluster = dbScan.getCluster();
	    if(data_cluster.size()<1)
                return;
	    if(data_cluster.size() == 1)
	    {
                for(auto i:data_cluster[0])
                       right_points.push_back({int(i.y),int(i.x)}); 
	    }
	    else
	    {
		for(int i=0;i<data_cluster.size()-1;++i)
		{
		   for(int j=i+1;j<data_cluster.size();++j)
		   {
		   	if(data_cluster[i].back().getDis(data_cluster[j].front())<(150*150))
			{
			  data_cluster[i].insert(data_cluster[i].end(),data_cluster[j].begin(),data_cluster[j].end());
	                  data_cluster.erase(data_cluster.begin()+(j--));
			}
		    }		
		}
		int index_max = 0;
		int max = data_cluster[0].size();
		for(int i = 1;i<data_cluster.size();++i)
		{
		    if(data_cluster[i].size()>=max)
		    {
			 max = data_cluster[i].size();
			 index_max = i; 
		    }    		
		}
		vector<int> row_max = row_col(data_cluster[index_max],out_img.rows+1); 
		data_cluster.erase(data_cluster.begin()+index_max);
		vector<int> row_min(row_max.size(),-1);
		if(data_cluster.size()<1)
		{
		    for(int i=0;i<row_max.size();++i)
			{
			    if(row_max[i]>=0)
			    	right_points.push_back({i,row_max[i]}); 			
			}			
		}
		else
		{
		     vector<vector<int>> data_rows(data_cluster.size());
            	     for(int i = 0;i<data_rows.size();i++)
            	     {
                	data_rows[i] = row_col(data_cluster[i],out_img.rows);
            	     }
		     for(auto i:data_rows)
		     {
			for(int j=0;j<i.size();++j)
			{
			   int dist_ = dist_x(row_max[j],i[j]);
			   if(row_max[j] >= 0  && i[j] >=0 && dist_>170 && dist_<240)
				row_min[j] = i[j];
			}	
		     }	
		     for(int i=0;i<row_max.size();++i)
		     {
			 if(row_max[i]>=0)
			     right_points.push_back({i,row_max[i]}); 	
			 if(row_min[i]>=0)
			     left_points.push_back({i,row_min[i]}); 			
		     }   			
		}
	    }
  	}

	bool regression_left()
    {
        if(left_points.size()<1)
            return false;
	    long sumX[5] = {0,0,0,0,0};
	    long sumY[3] = {0,0,0};
	    long pow2 = 0;
	    for(auto point=left_points.begin();point!=left_points.end();point++)
        {
		    pow2 = (point->x)*(point->x);
	        sumX[0]++;
		    sumX[1]+= (point->x);
		    sumX[2]+= pow2;
		    sumX[3]+= pow2*(point->x);
		    sumX[4]+= pow2*pow2;
		    sumY[0]+= (point->y);
		    sumY[1]+= (point->y)*(point->x);
		    sumY[2]+= (point->y)*pow2;
	    }	
        solve_system(sumX,sumY,polyleft);
        return true;
	}

    bool regression_right()
    {
        if(right_points.size()<1)
            return false;
	    long sumX[5] = {0,0,0,0,0};
	    long  sumY[3] = {0,0,0};
	    long  pow2 = 0;
	    for(auto point=right_points.begin();point!=right_points.end();point++)
        {
		    pow2 = (point->x)*(point->x);
	        sumX[0]++;
		    sumX[1]+= (point->x);
		    sumX[2]+= pow2;
		    sumX[3]+= pow2*(point->x);
		    sumX[4]+= pow2*pow2;
		    sumY[0]+= (point->y);
		    sumY[1]+= (point->y)*(point->x);
		    sumY[2]+= (point->y)*pow2;
	    }	
        solve_system(sumX,sumY,polyright);
        return true;
	}
 	void solve_system(long *sX,long *sY,float *x)
     {
	    int n,i,j,k;
        n=3;
    	float a[n][n+1];
        //declare an array to store the elements of augmented-matrix    
    	//"Enter the elements of the augmented-matrix row-wise
        a[0][0]=sX[0];   
        a[0][1]=sX[1];   
        a[0][2]=sX[2];   
        a[0][3]=sY[0];
        ////////////////   
        a[1][0]=sX[1];   
        a[1][1]=sX[2];   
        a[1][2]=sX[3];   
        a[1][3]=sY[1];  
        ////////////////   
        a[2][0]=sX[2];   
        a[2][1]=sX[3];   
        a[2][2]=sX[4];   
        a[2][3]=sY[2];   
        ////////////////
        for (i=0;i<n;i++)                    //Pivotisation
            for (k=i+1;k<n;k++)
                if (abs(a[i][i])<abs(a[k][i]))
                    for (j=0;j<=n;j++){
                        double temp=a[i][j];
                        a[i][j]=a[k][j];
                        a[k][j]=temp;
                    }
        for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
            for (k=i+1;k<n;k++){
                double t=a[k][i]/a[i][i];
                for (j=0;j<=n;j++)
                    a[k][j]=a[k][j]-t*a[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
        for (i=n-1;i>=0;i--)                //back-substitution
        {                        //x is an array whose values correspond to the values of x,y,z..
            x[i]=a[i][n];                //make the variable to be calculated equal to the rhs of the last equation
            for (j=i+1;j<n;j++)
                if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                    x[i]=x[i]-a[i][j]*x[j];
            x[i]=x[i]/a[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
        } 
	}

    void draw_lines(Mat &img)
    {
	    float columnL,columnL_aux;
		float columnR,columnR_aux;
	    float row;
	    float m = 0.0,b = 0.0,c=0.0;
		bool find_line_left;
		bool find_line_right;
		float angle_to_mid_radian;
		find_line_right = regression_right();
		find_line_left = regression_left();
		right_points.clear();
	    left_points.clear();
		center_cam =(img.cols/2)+1;
        if(find_line_left && find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright[0] + polyright[1]*(row)+polyright[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(255, 0, 0), 2);
                columnL = polyleft[0] + polyleft[1]*(row)+polyleft[2]*(row*row);
                circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(255, 0, 0), 2);
            }
            center_lines = (columnR + columnL)/2;
            distance_center = center_cam - center_lines;
		    if(distance_center==0)
			    angle = 90;
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
		    }
            line(img,Point(center_lines,0),Point(center_cam,(img.rows-1)),Scalar(0,0,255),2); 
            for(int k = 0;k < 3;k++)
            {
                polyleft_last[k] = polyleft[k]; 
                polyright_last[k] =polyright[k];
            }
        }
        else if(find_line_left)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnL = polyleft[0] + polyleft[1]*(row)+polyleft[2]*(row*row);
               circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(255, 0, 0),2);
            }
            //columnL = polyleft[0] + polyleft[1]*(0.0)+polyleft[2]*(0.0);
            columnL = polyleft[0];
            columnL_aux =  polyleft[0] + polyleft[1]*static_cast<float>(img.rows-1)+polyleft[2]*((img.rows-1)*(img.rows-1));
            if(columnL_aux == columnL)
            {
                angle = 90;
                center_lines = columnL_aux;
            }
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(columnL - columnL_aux ));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
                if(angle<90)
                {
                    angle_to_mid_radian =  (float)(angle)*0.0174533;
                    center_lines = center_cam+(int)(360.0*cos(angle_to_mid_radian));
                }
                else
                {
                    angle_to_mid_radian =  (float)(180-angle)*0.0174533;
                    center_lines = center_cam-(int)(360.0*cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
            
            for(int k = 0;k < 3;k++)
                polyleft_last[k] = polyleft[k]; 
        }
       else if(find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright[0] + polyright[1]*(row)+polyright[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(255, 0, 0), 2);
            }
            //columnR = polyright[0] + polyright[1]*(0.0)+polyright[2]*(0.0);
            columnR = polyright[0];
            columnR_aux = polyright[0] + polyright[1]*static_cast<float>(img.rows-1)+polyright[2]*static_cast<float>((img.rows-1)*(img.rows-1));
            if(columnR_aux == columnR)
            {
                angle = 90;
                center_lines = columnR_aux;
            }
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(columnR - columnR_aux ));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
                if(angle<90)
                {
                    angle_to_mid_radian =  angle*0.0174533;
                    center_lines = center_cam+(int)(360.0*cos(angle_to_mid_radian));
                }
                else
                {
                 angle_to_mid_radian =  (float)(180-angle)*0.0174533;
                    center_lines = center_cam-(int)(360.0*cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
             
            for(int k = 0;k < 3;k++)
                polyright[k] = polyright_last[k];
        }
        if(!find_line_left && !find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright_last[0] + polyright_last[1]*(row)+polyright_last[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(255, 0, 0), 2);
                columnL = polyleft_last[0] + polyleft_last[1]*(row)+polyleft_last[2]*(row*row);
                circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(255, 0, 0), 2);
            }
            center_lines = (columnR + columnL)/2;
            distance_center = center_cam - center_lines;
		    if(distance_center==0)
			    angle = 90;
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
		    }
            
        }
        line(img,Point(center_cam,(img.rows/4)),Point(center_cam,(img.rows*3/4)),Scalar(0,255,0),2); 
        line(img,Point(center_lines,0),Point(center_cam,(img.rows-1)),Scalar(0,0,255),2); 
    	ss.str(" ");
    	ss.clear();
        ss<<"[ANG]: "<<angle;
        putText(img, ss.str(), Point2f(2,20), 0,1, Scalar(0,255,255), 2);      
        }
};
int main(int argc,char **argv){
    ros::init(argc, argv, "line_detection");
    LineDetection *ic= new  LineDetection;
    ros::spin();
    return 0;
}