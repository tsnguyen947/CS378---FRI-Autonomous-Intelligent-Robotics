#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <fstream>
#include "ros/package.h"

using namespace std;
using namespace cv;

string relPath = ros::package::getPath("facial_recog");
vector<string> names;
Mat image;

void imgCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImageConstPtr temp = cv_bridge::toCvShare(msg);
	image = temp->image;
}

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
	std::ifstream file(filename.c_str(), ifstream::in);
	if (!file) {
		string error_message = "No valid input file was given, please check the given filename.";
		CV_Error(CV_StsBadArg, error_message);
	}
	string line, path, classlabel;
	while (getline(file, line)) {
		stringstream liness(line);
		getline(liness, path, separator);
		getline(liness, classlabel);
		if(!path.empty() && !classlabel.empty()) {
			images.push_back(imread(relPath + path, CV_LOAD_IMAGE_GRAYSCALE));
			labels.push_back(atoi(classlabel.c_str()));
		}
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "facial_recog");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Bool>("/facial_recog/visual", 1000);

	string haar = relPath + "/haarcascade_frontalface_alt.xml";
	string csv = relPath + "/images.txt";
	string face_recog = relPath + "/face_recog.xml";
	string nameData = relPath + "/nameData.txt";

	string data;

	ifstream file(nameData.c_str(), ifstream::in);
	if(file.is_open()){
		while(getline(file, data)){		
			stringstream dataStream(data);
			string name;

			dataStream >> name;

			names.push_back(name);
		}
		file.close();
	}
	else{
		cout << "cannot read nameData file" << endl;
	}

	ifstream input(face_recog.c_str(), ifstream::in); 	
	Ptr<FaceRecognizer> model = createLBPHFaceRecognizer(1, 8, 8, 8, 150.0);

	if(input.good()){ model->load(face_recog); }

	else{
		vector<Mat> images;
		vector<int> labels;
		try{
			read_csv(csv, images, labels);
		} 
		catch(cv::Exception& e) {
			cout << "cannot read csv file" << endl;
		}
		if(images.size() <= 1) {
			string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
			CV_Error(CV_StsError, error_message);
		}
		model->train(images, labels);
	}

	CascadeClassifier haar_cascade;
	if(!haar_cascade.load(haar)) { cout << "Could not load haar_cascade" << endl; }

	ros::Subscriber sub = n.subscribe("/nav_kinect/rgb/image_color", 1000, imgCallback);

	double ros_rate = 20.0;
	ros::Rate r(ros_rate);

	int lastPrediction = -2;
	std_msgs::Bool msg;

	while(ros::ok()){
		ros::spinOnce();
		r.sleep();

		if(!image.empty()){
			Mat gray;
			cvtColor(image, gray, CV_BGR2GRAY);

			vector<Rect> faces;
			haar_cascade.detectMultiScale(gray, faces, 1.1, 3, 0, Size(gray.size().height / 10, gray.size().width / 10), gray.size());

			if(faces.size() == 1){
				msg.data = true;
				pub.publish(msg);
				Rect face_i = faces[0];
				Mat face = gray(face_i);
				int prediction = model->predict(face);
				if(prediction != lastPrediction || prediction == -1){
					vector<cv::Mat> img;
					vector<int> label;
					if(prediction == -1){
						string new_name;
						string exit = "N";
						cout << "Hi, I don't think we've met. What's your name?" << endl;
						cout << "Enter your name or 'N' to skip." << endl;
						cin >> new_name;

						if(new_name.compare(exit) != 0){
							if(find(names.begin(), names.end(), new_name) != names.end()) {
								cout << "Oh, sorry I didn't recognize you." << endl;
								img.push_back(face);
								int index = find(names.begin(), names.end(), new_name) - names.begin();
								label.push_back(index);

								model->update(img, label);
							}
							else{
								cout << "Nice to meet you " << new_name << "." << endl;
								ofstream nameFile;
								nameFile.open(nameData.c_str(), std::ios_base::app);
								nameFile << new_name << "\n";
								nameFile.close();

								img.push_back(face);
								label.push_back(names.size());

								model->update(img, label);

								names.push_back(new_name);
							}
						}
					}
					else{
						cout<< "Hi " << names.at(prediction) << "." << endl;
					}
				}
				lastPrediction = prediction;
			}
			else{
				msg.data = false;
				pub.publish(msg);
			}
		}
		image.release();
	}
	model->save(face_recog);
}
