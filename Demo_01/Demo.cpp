//============================================================================
// Name        : Demo.h
// Author      : Amos Makendi
// Version     : 1.0
// Copyright   : -
// Description : 
//============================================================================
#include "Demo.h"
#include<opencv2/core.hpp>
#include<opencv2/core/mat.hpp>
#include<opencv2/core/matx.hpp>

// namespace is a method for preventing name conflicts in large projects
namespace demo {

/**
* @brief get the conditioning matrix of given points
* @param  the points as matrix
* @returns the conditioned matrix
*/
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f> &points)
{
    // compute tx, ty out of the point
    float tX = 0, tY = 0;
    float sX = 0, sY = 0;
    float sumX = 0.f, sumY = 0.f;


    for (int i=0; i < points.size();i++){
        sumX += points[i][0];
        sumY += points[i][1];

    }

    tX = sumX / points.size();
    tY = sumY / points.size();
   
   // compute (sx,sy) as the mean distance to the origin 
   for (int j = 0; j < points.size(); j++){
       sX = sX + fabs(points[j][0] - tX);
       sY = sY + fabs(points[j][1] - tY);
   }

   sX = sX / points.size();
   sY = sY / points.size(); 

    // get the conditioning  matrix
    cv::Matx33f result = {1/sX, 0, -tX/sX, 0, 1/sY, -tY/sY, 0, 0, 1};

    


    return result;
}



/**
 * @brief define the design matrix as needed to compute 2D-homography
 * @param conditioned_base first set of conditioned points x' --> x' = H * x
 * @param conditioned_attach second set of conditioned points x --> x' = H * x
 * @returns the design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach)
{
    
    cv::Mat_<float> A = cv::Mat_<float>::zeros(8, 9);

    // create design matrix from point Pi
    for (int i = 0; i < conditioned_base.size(); i++){

        A.at<float>(2*i,0) = -conditioned_attach[i][0];
        A.at<float>(2*i,1) = -conditioned_attach[i][1];
        A.at<float>(2*i,2) = -1;
        A.at<float>(2*i,6) = conditioned_attach[i][0] * conditioned_base[i][0];
        A.at<float>(2*i,7) = conditioned_attach[i][1] * conditioned_base[i][0];
        A.at<float>(2*i,8) = conditioned_base[i][0];

        A.at<float>(2*i+1,3) = -conditioned_attach[i][0];
        A.at<float>(2*i+1,4) = -conditioned_attach[i][1];
        A.at<float>(2*i+1,5) = -1;
        A.at<float>(2*i+1,6) = conditioned_attach[i][0] * conditioned_base[i][1];
        A.at<float>(2*i+1,7) = conditioned_attach[i][1] * conditioned_base[i][1];
        A.at<float>(2*i+1,8) = conditioned_base[i][1];

    }

    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Design Matrix A:\n" << A  <<std::endl;
    std::cout<<"--------------------------------"<<std::endl;

    return A;
}


/**
 * @brief solve homogeneous equation system by usage of SVD
 * @param A the design matrix
 * @returns solution of the homogeneous equation system
 */
cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A)
{
    // svd decomposition
    cv::SVD svd(A, cv::SVD::Flags::FULL_UV); 

    // h vector of unknowns
    cv::Mat_<float> h = svd.vt.row(8);

    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Vector_unknows h:\n"<< h <<std::endl;
    std::cout<<"--------------------------------"<<std::endl;

    // reshape homography H from h-vector of unknowns
    cv::Matx33f H( h.at<float>(0, 0), h.at<float>(0, 1), h.at<float>(0, 2), 
                   h.at<float>(0, 3), h.at<float>(0, 4), h.at<float>(0, 5),
                   h.at<float>(0, 6), h.at<float>(0, 7), h.at<float>(0, 8));

    // display estimate homography
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"H_hat before deconditioning:\n"<<H<<std::endl;
    std::cout<<"--------------------------------"<<std::endl;
   
    return H;    
}


/**
 * @brief decondition a homography that was estimated from conditioned point clouds
 * @param T_base conditioning matrix T' of first set of points x'
 * @param T_attach conditioning matrix T of second set of points x
 * @param H conditioned homography that has to be un-conditioned (in-place)
 */
cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H) 
{

    // print getcondiontioning 
    std::cout<<"------------------------------"<<std::endl;
    std::cout<<"T_base matrix:\n"<<T_base<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"T_base_inv():\n"<<T_base.inv()<<std::endl;
    std::cout<<"------------------------------"<<std::endl;
    std::cout<<"T_attach matrix:\n"<<T_attach<<std::endl;
    std::cout<<"------------------------------"<<std::endl;
    std::cout<<"H_hat before conditioning:\n"<<H<<std::endl;
    std::cout<<"------------------------------"<<std::endl;
    
    cv::Matx33f  reverseConditioning_H = T_base.inv() * H * T_attach;

    // display deconditioned homography
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"H_hat after deconditioning:\n"<<reverseConditioning_H<<std::endl;
    std::cout<<"--------------------------------"<<std::endl;

    // final transformation matrix is equal to euclidian normalize
    cv::Matx33f H_final = reverseConditioning_H / H(2, 2);

    // display euclidean normalized homography
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"H_final euclidean normalized:\n"<<H_final<<std::endl;
    std::cout<<"--------------------------------"<<std::endl;


    return H_final;
}


/**
 * @brief compute the 2D homography
 * @param base first set of points x'
 * @param attach second set of points x
 * @returns homography H, so that x' = Hx
 */
cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach)
{
    // condition corresponding points from the initial and current images to avoid ill-conditioned 
    // ill-conditioned points lead to numerical instabillity 
    
    cv::Matx33f condbase_T = getCondition2D(base);
    cv::Matx33f condattach_T = getCondition2D(attach);

    // apply
    std::vector<cv::Vec3f> basenew = applyH_2D(base, condbase_T, GEOM_TYPE_POINT);
    std::vector<cv::Vec3f> attachnew = applyH_2D(attach, condattach_T, GEOM_TYPE_POINT);

    // compute design matrix
    cv::Mat_<float> desmat = getDesignMatrix_homography2D( basenew ,  attachnew );

    // solve system of equation A.h = 0 using SVD
    cv::Matx33f dltnew = solve_dlt_homography2D(desmat);

    // estimate homography
    cv::Matx33f H_hat = decondition_homography2D(condbase_T, condattach_T, dltnew);

    return H_hat;
}




/**
 * @brief Applies a 2D transformation to an array of points or lines
 * @param H Matrix representing the transformation
 * @param geomObjects Array of input objects, each in homogeneous coordinates
 * @param type The type of the geometric objects, point or line. All are the same type.
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type)
{
    std::vector<cv::Vec3f> result;

    switch (type) {
        case GEOM_TYPE_POINT: {
            for (int i = 0; i<geomObjects.size(); i++){
                result.push_back(H*geomObjects[i]);
            }
        } break;
        case GEOM_TYPE_LINE: {
            for (int i = 0; i <geomObjects.size();i++){
                result.push_back(H.inv().t() * geomObjects[i]);
            }
        } break;
        default:
            throw std::runtime_error("Unhandled geometry type!");
    }
    return result;
}


/**
 * @brief Convert a 2D point from Euclidean to homogeneous coordinates
 * @param p The point to convert (in Euclidean coordinates)
 * @returns The same point in homogeneous coordinates
 */
cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p)
{
    cv::Vec3f homoCord;
    homoCord[0] = p[0];
    homoCord[1] = p[1];
    homoCord[2] = 1;
    
    return homoCord;
}

}




















