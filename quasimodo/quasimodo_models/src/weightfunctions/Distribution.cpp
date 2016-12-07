#include "weightfunctions/Distribution.h"

namespace reglib
{
Distribution::Distribution(){
    debugg_print = false;
}
Distribution::~Distribution(){}
void	Distribution::reset(){printf("%s in %s\n",__PRETTY_FUNCTION__,__FILE__);}
void    Distribution::train(std::vector<float> & hist, unsigned int nr_bins){   printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
void    Distribution::update(){                                                 printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
double  Distribution::getval(double x){                                         printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
double  Distribution::getcdf(double x){                                         printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
void    Distribution::print(){                                                  printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
void    Distribution::setRegularization(double x){regularization = x;update();}
double  Distribution::getNoise(){return 1;}
void    Distribution::setNoise(double x){                                       printf("%s in %s not implemented, stopping\n",__PRETTY_FUNCTION__,__FILE__);exit(0);}
void    Distribution::getMaxdMind(double & maxd, double & mind, double prob){
    if(debugg_print){printf("%s in %s\n",__PRETTY_FUNCTION__,__FILE__);}
	double midval = getval(mean);
	double half = prob*0.5;
	double minDist = 0;
	double minDistScore = getval(mean+minDist)/midval;
	double maxDist = 1;
	double maxDistScore = getval(mean+maxDist)/midval;

//    double target = 0.000001;//1-prob;
	//Grow to find interval
    while(maxDistScore > prob){
		minDist = maxDist;
		minDistScore = maxDistScore;
		maxDist *= 2;
		maxDistScore = getval(mean+maxDist)/midval;
//		if(debugg_print){printf("%f %f %f\n",minDist,maxDist,maxDistScore);}
	}

	//bisect to opt
	for(unsigned int it = 0; it < 40; it++){
		double midDist = (minDist+maxDist)*0.5;
		double midDistScore = getval(mean+midDist)/midval;//getcdf(mean+midDist);

//		if(debugg_print){printf("[%7.7f %7.7f] -> %7.7f\n",minDist,maxDist,midDistScore);}

        if(midDistScore < prob){
			maxDist = midDist;
			maxDistScore = midDistScore;
		}else{
			minDist = midDist;
			minDistScore = midDistScore;
		}

        if(fabs(maxDistScore-minDistScore) < prob*0.01){break;}
	}


/*
	double half = prob*0.5;
    double minDist = 0;
	double minDistScore = getcdf(mean+minDist);
    double maxDist = 1;
    double maxDistScore = getcdf(mean+maxDist);

    double target = 1-half;
    //Grow to find interval
    while(maxDistScore < target){
        minDist = maxDist;
        minDistScore = maxDistScore;
        maxDist *= 2;
		maxDistScore = getcdf(mean+maxDist);
	}

    //bisect to opt
	for(unsigned int it = 0; it < 40; it++){
        double midDist = (minDist+maxDist)*0.5;
		double midDistScore = getcdf(mean+midDist);
        if(midDistScore > target){
            maxDist = midDist;
            maxDistScore = midDistScore;
        }else{
            minDist = midDist;
            minDistScore = midDistScore;
        }
        if((maxDistScore-minDistScore) < 1e-15){break;}
    }
*/
    double midDist = (minDist+maxDist)*0.5;
    mind = mean-midDist;
	maxd = mean+midDist;
}

}
