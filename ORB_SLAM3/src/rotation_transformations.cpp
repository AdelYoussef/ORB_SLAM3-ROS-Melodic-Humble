
#include<rotation_transformations.h>

using namespace std;


void getQuaternion(cv::Mat R, float (&Q)[4])
{
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);

    if (trace > 0.0) 
    {
        float s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}


cv::Mat rot2euler(const cv::Mat & R , string order)
{
    

    cv::Mat euler(3, 1, CV_64F);
    float x, y, z;

    if(order ==  "ZYX")
    {
        float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            
            z = atan2(R.at<float>(1,0), R.at<float>(0,0)); // phi
            y = atan2(-R.at<float>(2,1), sy); //theta
            x = atan2(R.at<float>(2,1) , R.at<float>(2,2)); //psi
        }
        else
        {
            z = 0; // phi
            y = atan2(-R.at<float>(2,0), sy);//theta
            x = atan2(-R.at<float>(1,2), R.at<float>(1,1)); //psi
            
        }
    }

    if(order ==  "ZXY")
    {
        float sy = sqrt(R.at<float>(1,1) * R.at<float>(1,1) +  R.at<float>(0,1) * R.at<float>(0,1) );
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(0,1), R.at<float>(1,1)); // phi
            y = -atan2(-R.at<float>(2,1), sy); //theta
            x = -atan2(R.at<float>(2,1) , R.at<float>(2,2)); //psi
            
        }
        else
        {
            z = 0; // phi
            y = -atan2(-R.at<float>(2,1), sy); //theta
            x = -atan2(-R.at<float>(0,2), R.at<float>(0,0)); //psi
            
        }
    }

    if(order == "YXZ")
    {
        float sy = sqrt(R.at<float>(2,2) * R.at<float>(2,2) +  R.at<float>(0,2) * R.at<float>(0,2));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = atan2(R.at<float>(0,2), R.at<float>(2,2)); // phi
            y = atan2(-R.at<float>(1,2), sy); //theta
            x = atan2(R.at<float>(1,0) , R.at<float>(1,1)); //psi

        }
        else
        {
            z = 0; // phi
            y = atan2(-R.at<float>(1,2), sy);  //theta
            x = atan2(-R.at<float>(0,1), R.at<float>(0,0)); //psi

        }
    }

    if(order == "YZX")
    {
        float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(2,0) * R.at<float>(2,0));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(2,0), R.at<float>(0,0)); // phi
            y = -atan2(-R.at<float>(1,0), sy); //theta
            x = -atan2(R.at<float>(1,2) , R.at<float>(1,1)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(-R.at<float>(1,0), sy);  //theta
            x = -atan2(-R.at<float>(2,1), R.at<float>(2,2)); //psi

        }
    }

    if(order == "XYZ")
    {
        float sy = sqrt(R.at<float>(2,2) * R.at<float>(2,2) +  R.at<float>(1,2) * R.at<float>(1,2));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(1,2), R.at<float>(2,2)); // phi
            y = -atan2(-R.at<float>(0,2), sy); //theta
            x = -atan2(R.at<float>(0,1) , R.at<float>(0,0)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(-R.at<float>(0,2), sy);  //theta
            x = -atan2(-R.at<float>(1,0), R.at<float>(1,1)); //psi

        }
    }

    if(order == "XZY")
    {
        float sy = sqrt(R.at<float>(1,1) * R.at<float>(1,1) +  R.at<float>(2,1) * R.at<float>(2,1));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = atan2(R.at<float>(2,1), R.at<float>(1,1)); // phi
            y = atan2(-R.at<float>(0,1), sy); //theta
            x = atan2(R.at<float>(0,2) , R.at<float>(0,0)); //psi

        }
        else
        {
            z = 0; // phi
            y = atan2(-R.at<float>(0,1), sy);  //theta
            x = atan2(-R.at<float>(2,0), R.at<float>(2,2)); //psi

        }
    }

    if(order == "ZYZ")
    {
        float sy = sqrt(R.at<float>(2,1) * R.at<float>(2,1) +  R.at<float>(2,0) * R.at<float>(2,0));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(1,2), -R.at<float>(0,2)); // phi
            y = -atan2( sy, R.at<float>(2,2)); //theta
            x = -atan2(R.at<float>(2,1) , R.at<float>(2,0)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(sy , R.at<float>(2,2));  //theta
            x = -atan2(-R.at<float>(1,0), R.at<float>(1,1)); //psi

        }
    }


    if(order == "ZXZ")
    {
        float sy = sqrt(R.at<float>(2,1) * R.at<float>(2,1) +  R.at<float>(2,0) * R.at<float>(2,0));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = atan2(R.at<float>(0,2), -R.at<float>(1,2)); // phi
            y = atan2( sy, R.at<float>(2,2)); //theta
            x = atan2(R.at<float>(2,1) , R.at<float>(2,1)); //psi

        }
        else
        {
            z = 0; // phi
            y = atan2(sy , R.at<float>(2,2));  //theta
            x = atan2(-R.at<float>(0,1), R.at<float>(0,0)); //psi

        }
    }


    if(order == "YZY")
    {
        float sy = sqrt(R.at<float>(1,2) * R.at<float>(1,2) +  R.at<float>(1,0) * R.at<float>(1,0));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = atan2(R.at<float>(2,1), -R.at<float>(0,1)); // phi
            y = atan2( sy, R.at<float>(1,1)); //theta
            x = atan2(R.at<float>(1,2) , R.at<float>(1,0)); //psi

        }
        else
        {
            z = 0; // phi
            y = atan2(sy , R.at<float>(1,1));  //theta
            x = atan2(-R.at<float>(2,0), R.at<float>(2,2)); //psi

        }
    }

    if(order == "YXY")
    {
        float sy = sqrt(R.at<float>(1,2) * R.at<float>(1,2) +  R.at<float>(1,0) * R.at<float>(1,0));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(0,1), -R.at<float>(2,1)); // phi
            y = -atan2( sy, R.at<float>(1,1)); //theta
            x = -atan2(R.at<float>(1,0) , R.at<float>(1,2)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(sy , R.at<float>(1,1));  //theta
            x = -atan2(-R.at<float>(0,2), R.at<float>(0,0)); //psi

        }
    }

    if(order == "XZX")
    {
        float sy = sqrt(R.at<float>(0,2) * R.at<float>(0,2) +  R.at<float>(0,1) * R.at<float>(0,1));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(2,0), -R.at<float>(1,0)); // phi
            y = -atan2( sy, R.at<float>(0,0)); //theta
            x = -atan2(R.at<float>(0,2) , R.at<float>(0,1)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(sy , R.at<float>(0,0));  //theta
            x = -atan2(-R.at<float>(2,1), R.at<float>(2,2)); //psi

        }
    }


    if(order == "XZX")
    {
        float sy = sqrt(R.at<float>(0,1) * R.at<float>(0,1) +  R.at<float>(0,1) * R.at<float>(0,1));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = -atan2(R.at<float>(2,0), -R.at<float>(1,0)); // phi
            y = -atan2( sy, R.at<float>(0,0)); //theta
            x = -atan2(R.at<float>(0,2) , R.at<float>(0,1)); //psi

        }
        else
        {
            z = 0; // phi
            y = -atan2(sy , R.at<float>(0,0));  //theta
            x = -atan2(-R.at<float>(2,1), R.at<float>(2,2)); //psi

        }
    }

    if(order == "XYX")
    {
        float sy = sqrt(R.at<float>(0,1) * R.at<float>(0,1) +  R.at<float>(0,2) * R.at<float>(0,2));
        bool singular = sy < 1e-6; 
        
        if (!singular)
        {
            z = atan2(R.at<float>(1,0), -R.at<float>(2,0)); // phi
            y = atan2(sy, R.at<float>(0,0)); //theta
            x = atan2(R.at<float>(0,1) , R.at<float>(0,2)); //psi

        }
        else
        {
            z = 0; // phi
            y = atan2(sy , R.at<float>(0,0));  //theta
            x = atan2(-R.at<float>(1,2), R.at<float>(1,1)); //psi

        }
    }

    


    euler.at<double>(0) = x * 180 / 3.14;
    euler.at<double>(1) = y * 180 / 3.14;
    euler.at<double>(2) = z * 180 / 3.14;
    return euler;

}
