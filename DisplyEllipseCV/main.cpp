#include <iostream>
#include <opencv2/opencv.hpp>
#include <halconcpp/HalconCpp.h>

namespace tlm
{
    struct Point2DInfo
    {
        double x;
        double y;
        int Code;
        int ID;
        double lAxis;
        double sAxis;
        double dAngle;
        double ovality;
    };

    struct EllipseCoeff
    {
        double a;
        double b;
        double c;
        double d;
        double e;
    };

    int DetectEllipseWithHalcon(const HalconCpp::HObject& dst, std::vector<Point2DInfo>& vecPt2d, std::vector<std::vector<cv::Point2f>>& vvEdige);
    bool HalconImage2CV(HalconCpp::HObject& halconImage, cv::Mat& CVImage);
    bool CVImage2Halcom(cv::Mat& CVImage, HalconCpp::HObject& halconImage);

    //输入的角度参数需要为逆时针弧度制
    bool EllipseParam2FunctionCoeff(const cv::Point2f& center, const cv::Size2f& axis, const double& angle, EllipseCoeff& ellipseCoeff);
    bool EllipseParam2FunctionCoeff(const Point2DInfo& PointWithEllipse, EllipseCoeff& ellipseCoeff);

    //获得的椭圆的角度为逆时针弧度制   
    bool EllipseFunctionCoeff2Param(const EllipseCoeff& ellipseCoeff, cv::Point2f& center, cv::Size2f& axis, double& angle/*弧度*/);
    bool EllipseFunctionCoeff2Param(const EllipseCoeff& ellipseCoeff, Point2DInfo& PointWithEllipse);
    bool EllipseCoeff2FunctionMatrix(const EllipseCoeff& ellipseCoeff, cv::Mat& ellipseMatrix);
    bool EllipseFunctionMatrix2Coeff(const cv::Mat& ellipseMatrix, EllipseCoeff& ellipseCoeff);
    bool FitEllipse(const std::vector<cv::Point2f>& vecEdgePt, EllipseCoeff& ellipCoeff);       //用于验证椭圆参数转换函数是否正确
    
    //用于椭圆验证，椭圆参数中的角度为逆时针弧度制
    bool EdgeSampling(const EllipseCoeff& coeff, const int nPointsNum, std::vector<cv::Point2f>& vecSamplingPts, std::vector<cv::Vec3f>* vecNs = NULL);
    bool EdgeSampling(const tlm::Point2DInfo& ptwithE, const int nPointsNum, std::vector<cv::Point2f>& vecSamplingPts, std::vector<cv::Vec3f>* vecNs = NULL);
    bool EdgeSampling(const cv::Point2f center, const cv::Size& axis, const double& angle, const int nPointsNum, std::vector<cv::Point2f>& vecSamplingPts,
        std::vector<cv::Vec3f>* vecNs = NULL);
    
    //计算直线与椭圆的交点
    bool LineAndEllipseIntersection(const cv::Vec3f& lineCoeff, EllipseCoeff& ellipseCoeff, std::vector<cv::Point2f>& intersections);
    bool LineAndEllipseIntersection(const double la, const double lb, const double lc, EllipseCoeff& ellipseCoeff, std::vector<cv::Point2f>& intersections);
    bool LineAndEllipseIntersection(const cv::Vec3f& lineCoeff, const cv::Point2f& center, const cv::Size& axis, const double& angle, 
        std::vector<cv::Point2f>& intersections);
    bool LineAndEllipseIntersection(const double la, const double lb, const double lc, const cv::Point2f& center, const cv::Size& axis, const double& angle,
        std::vector<cv::Point2f>& intersections);
    void drawLineWithABC(cv::Mat& Image, const double a, const double b, const double c, const cv::Scalar& color);
}

int main()
{
    //所有角度以OpenCV绘制椭圆为准，即halcon中椭圆检测时获得角度的方向是反的，角度以弧度制保存
    //边缘点拟合函数，获得的椭圆的角度为逆时针弧度制
    //Dete
	cv::Mat img(600, 800, CV_8UC1,cv::Scalar(255));
	cv::ellipse(img, cv::Point(400, 300), cv::Size(200, 150), 120, 0, 360, cv::Scalar(0, 0, 255), -1);   //绘制椭圆
    HalconCpp::HObject himg;
    tlm::CVImage2Halcom(img, himg);
    std::vector<tlm::Point2DInfo> vecPt2D;
    std::vector<std::vector<cv::Point2f>> vvEdge;
    tlm::DetectEllipseWithHalcon(himg, vecPt2D, vvEdge);                //获得的椭圆的角度为逆时针弧度制
    tlm::EllipseCoeff coeffwithFit, coeffwithConvert, coeffwithConvert1;
    tlm::FitEllipse(vvEdge[0], coeffwithFit);                           //拟合椭圆获得椭圆方程系数
    std::vector<cv::Point2f> samplingPts;
    tlm::EdgeSampling(coeffwithFit, 100, samplingPts);
    tlm::drawLineWithABC(img, 1, -2, 0, cv::Scalar(0, 0, 255));
    std::vector<cv::Point2f> intersections;
    tlm::LineAndEllipseIntersection(cv::Vec3f(1, -2, 0), coeffwithFit, intersections);
    tlm::Point2DInfo elParamfit, elParamcon;
    tlm::EllipseFunctionCoeff2Param(coeffwithFit, elParamfit);          //获得的椭圆的角度为逆时针弧度制
    tlm::EllipseParam2FunctionCoeff(vecPt2D[0], coeffwithConvert);      //输入的椭圆参数需要为逆时针弧度制
    cv::Point2f center1 = cv::Point2f(vecPt2D[0].x, vecPt2D[0].y);
    cv::Size2f axis1 = cv::Size2f(vecPt2D[0].lAxis, vecPt2D[0].sAxis);
    double angle1 = vecPt2D[0].dAngle;
    tlm::EllipseParam2FunctionCoeff(center1, axis1, angle1, coeffwithConvert1);
    tlm::EllipseFunctionCoeff2Param(coeffwithConvert, elParamcon);      //将转换的椭圆系数转换成椭圆参数
    cv::Point2f center2; cv::Size2f axis2; double angle2;
    tlm::EllipseFunctionCoeff2Param(coeffwithConvert, center2, axis2, angle2);
    double anglefit, anglecon;
    anglefit = elParamfit.dAngle * 180.0 / PI;
    anglecon = elParamcon.dAngle * 180.0 / PI;
    cv::Mat coeffMatrix;
    tlm::EllipseCoeff2FunctionMatrix(coeffwithConvert, coeffMatrix);

    if (1 != vecPt2D.size())
    {
        return -1;
    }
    cv::Point2f center = cv::Point2f(vecPt2D[0].x, vecPt2D[0].y);   //400,300
    cv::Size axis = cv::Size(vecPt2D[0].lAxis, vecPt2D[0].sAxis);   //200,150
    double dAngle = 180 - vecPt2D[0].dAngle * 180 / PI;
	return 0;
}

namespace tlm
{
    int DetectEllipseWithHalcon(const HalconCpp::HObject& dst, std::vector<Point2DInfo>& vecPt2d, std::vector<std::vector<cv::Point2f>>& vvEdige)
    {
        vecPt2d.clear();

        HalconCpp::HObject  Edges, SelectedXLD, UnionContours1, ClosedContours, SelectedXLD1, UnionContours, SelectedXLD12;
        HalconCpp::HTuple  Phi, Radius1, Radius2, StartPhi1, EndPhi1, PointOrder1, Row1, Column1;

        EdgesSubPix(dst, &Edges, "canny", 1, 20, 40);
        SelectShapeXld(Edges, &SelectedXLD, "contlength", "and", 10, 5000);
        UnionCocircularContoursXld(SelectedXLD, &UnionContours1, 0.5, 0.1, 0.2, 30, 10, 10, "true", 1);
        CloseContoursXld(UnionContours1, &ClosedContours);
        SelectShapeXld(ClosedContours, &SelectedXLD1, "circularity", "and", 0.4, 1);
        UnionAdjacentContoursXld(SelectedXLD1, &UnionContours, 10, 1, "attr_keep");
        SelectShapeXld(UnionContours, &SelectedXLD12, "contlength", "and", 30, 16000);
        FitEllipseContourXld(SelectedXLD12, "fitzgibbon", -1, 0, 0, 200, 3, 2, &Row1,
            &Column1, &Phi, &Radius1, &Radius2, &StartPhi1, &EndPhi1, &PointOrder1);
        int nPtCount = Radius1.Length();

        //存储数据并返回
        for (int i = 0; i < nPtCount; i++)
        {
            HalconCpp::HObject HoneShape = SelectedXLD12.SelectObj(i+1);
            HalconCpp::HTuple Hrows, Hcols;
            HalconCpp::GetContourXld(HoneShape, &Hrows, &Hcols);
            std::vector<cv::Point2f> vecEdigePt;
            for (int j = 0; j < Hrows.Length(); j++)
            {
                cv::Point2f pt2d;
                pt2d.x = Hcols[j];
                pt2d.y = Hrows[j];
                vecEdigePt.push_back(pt2d);
            }
            vvEdige.push_back(vecEdigePt);
            Point2DInfo Pt2D;
            Pt2D.x = Column1[i].D();
            Pt2D.y = Row1[i].D();
            Pt2D.Code = 0;
            Pt2D.ID = 0;
            Pt2D.lAxis = Radius1[i].D();
            Pt2D.sAxis = Radius2[i].D();
            Pt2D.dAngle = PI - Phi[i].D();
            Pt2D.ovality = 1;
            vecPt2d.push_back(Pt2D);
        }
        return 1;
    }

    bool HalconImage2CV(HalconCpp::HObject& halconImage, cv::Mat& CVImage)
    {
        if (halconImage == NULL)
        {
            return false;
        }

        HalconCpp::HTuple htChannels;
        HalconCpp::HTuple     width, height;
        width = height = 0;
        //转换图像格式  
        HalconCpp::ConvertImageType(halconImage, &halconImage, "byte");
        CountChannels(halconImage, &htChannels);
        HalconCpp::HTuple cType;
        HalconCpp::HTuple grayVal;

        if (htChannels.I() == 1)
        {
            GetImageSize(halconImage, &width, &height);

            CVImage = cv::Mat(height, width, CV_8UC1);
            CVImage = cv::Mat::zeros(height, width, CV_8UC1);

            for (int i = 0; i < height.I(); ++i)
            {
                for (int j = 0; j < width.I(); ++j)
                {
                    GetGrayval(halconImage, i, j, &grayVal);
                    CVImage.at<uchar>(i, j) = (uchar)grayVal.I();
                }
            }
        }
        else if (htChannels.I() == 3)
        {
            GetImageSize(halconImage, &width, &height);
            CVImage = cv::Mat(height, width, CV_8UC3);
            for (int row = 0; row < height.I(); row++)
            {
                for (int col = 0; col < width.I(); col++)
                {
                    GetGrayval(halconImage, row, col, &grayVal);
                    CVImage.at<uchar>(row, col * 3) = (uchar)grayVal[2].I();
                    CVImage.at<uchar>(row, col * 3 + 1) = (uchar)grayVal[1].I();
                    CVImage.at<uchar>(row, col * 3 + 2) = (uchar)grayVal[0].I();
                }
            }

        }
        return true;
    }

    bool CVImage2Halcom(cv::Mat& CVImage, HalconCpp::HObject& halconImage)
    {
        if (CVImage.empty())
        {
            return false;
        }

        if (3 == CVImage.channels())
        {
            cv::Mat pImageRed, pImageGreen, pImageBlue;
            std::vector<cv::Mat> sbgr(3);
            cv::split(CVImage, sbgr);

            int length = CVImage.rows * CVImage.cols;
            uchar* dataBlue = new uchar[length];
            uchar* dataGreen = new uchar[length];
            uchar* dataRed = new uchar[length];

            int height = CVImage.rows;
            int width = CVImage.cols;
            for (int row = 0; row < height; row++)
            {
                uchar* ptr = CVImage.ptr<uchar>(row);
                for (int col = 0; col < width; col++)
                {
                    dataBlue[row * width + col] = ptr[3 * col];
                    dataGreen[row * width + col] = ptr[3 * col + 1];
                    dataRed[row * width + col] = ptr[3 * col + 2];
                }
            }
            HalconCpp::GenImage3(&halconImage, "byte", width, height, (Hlong)(dataRed), (Hlong)(dataGreen), (Hlong)(dataBlue));
            delete[] dataRed;
            delete[] dataGreen;
            delete[] dataBlue;
        }
        else if (1 == CVImage.channels())
        {
            int height = CVImage.rows;
            int width = CVImage.cols;
            uchar* dataGray = new uchar[width * height];
            memcpy(dataGray, CVImage.data, width * height);
            HalconCpp::GenImage1(&halconImage, "byte", width, height, (Hlong)(dataGray));
            delete[] dataGray;
        }
        return true;
    }

    bool EllipseParam2FunctionCoeff(const cv::Point2f& center, const cv::Size2f& axis, const double& angle, EllipseCoeff& ellipseCoeff)
    {
        float x0 = center.x;
        float y0 = center.y;
        float a = axis.width;
        float b = axis.height;
        float theta = angle;    //弧度

        if (a == 0 || b == 0)
            return false;
        float A = pow(a * sin(theta), 2) + pow(b * cos(theta), 2);
        float B = 2 * (b * b - a * a) * sin(theta) * cos(theta);
        float C = pow(a * cos(theta), 2) + pow(b * sin(theta), 2);
        float D = -2 * A * x0 - B * y0;
        float E = -B * x0 - 2 * C * y0;
        float F = -1.0 / 2.0 * (D * x0 + E * y0) - a * a * b * b;

        ellipseCoeff.a = A / F;
        ellipseCoeff.b = B / F;
        ellipseCoeff.c = C / F;
        ellipseCoeff.d = D / F;
        ellipseCoeff.e = E / F;

        return true;
    }

    bool EllipseParam2FunctionCoeff(const Point2DInfo& PointWithEllipse, EllipseCoeff& ellipseCoeff)
    {
        if (PointWithEllipse.lAxis == 0 || PointWithEllipse.sAxis == 0)
            return false;
        cv::Point2f center = cv::Point2f(PointWithEllipse.x, PointWithEllipse.y);
        cv::Size2f axis = cv::Size2f(PointWithEllipse.lAxis, PointWithEllipse.sAxis);
        double angle = PointWithEllipse.dAngle;
        EllipseParam2FunctionCoeff(center, axis, angle, ellipseCoeff);
        return true;
    }

    bool EllipseFunctionCoeff2Param(const EllipseCoeff& ellipseCoeff, cv::Point2f& center, cv::Size2f& axis, double& angle)
    {
        float A = ellipseCoeff.a;
        float B = ellipseCoeff.b;
        float C = ellipseCoeff.c;
        float D = ellipseCoeff.d;
        float E = ellipseCoeff.e;
        if (A == 0 && B == 0 && C == 0 && D == 0 && E == 0)
            return false;
        float fDelta = pow(B, 2) - 4 * A * C;
        if (fDelta >= 0)
            return false;
        center.x = (B * E - 2 * C * D) / (4 * A * C - B * B);
        center.y = (B * D - 2 * A * E) / (4 * A * C - B * B);
        float fAxis1 = 0, fAxis2 = 0;
        fAxis1 = sqrt(2 * (A * center.x * center.x + C * center.y * center.y + B * center.x * center.y - 1) / 
            (A + C - sqrt(pow(A - C, 2) + B * B)));
        fAxis2 = sqrt(2 * (A * center.x * center.x + C * center.y * center.y + B * center.x * center.y - 1) /
            (A + C + sqrt(pow(A - C, 2) + B * B)));
        if (fAxis1 > fAxis2)
        {
            axis.width = fAxis1;
            axis.height = fAxis2;
        }
        else
        {
            axis.width = fAxis2;
            axis.height = fAxis1;
        }
        angle = 1.0 / 2.0 * atan(B / (A - C));      //弧度

        if (abs(A) > abs(C))
        {
            angle = angle + PI/2;
        }
        if (angle < 0)
        {
            angle = angle + PI;
        }

        return true;
    }

    bool EllipseFunctionCoeff2Param(const EllipseCoeff& ellipseCoeff, Point2DInfo& PointWithEllipse)
    {
        if (ellipseCoeff.a == 0 || ellipseCoeff.b == 0)
            return false;
        cv::Point2f center;
        cv::Size2f axis;
        double angle;
        EllipseFunctionCoeff2Param(ellipseCoeff, center, axis, angle);
        PointWithEllipse.x = center.x;
        PointWithEllipse.y = center.y;
        PointWithEllipse.lAxis = axis.width;
        PointWithEllipse.sAxis = axis.height;
        PointWithEllipse.dAngle = angle;
        return true;
    }

    bool EllipseCoeff2FunctionMatrix(const EllipseCoeff& ellipseCoeff, cv::Mat& ellipseMatrix)
    {
        if (ellipseCoeff.a == 0)
        {
            return FALSE;
        }
        ellipseMatrix = cv::Mat::eye(3, 3, CV_32FC1);
        ellipseMatrix.ptr<float>(0)[0] = ellipseCoeff.a;
        ellipseMatrix.ptr<float>(0)[1] = ellipseCoeff.b / 2;
        ellipseMatrix.ptr<float>(1)[0] = ellipseCoeff.b / 2;
        ellipseMatrix.ptr<float>(1)[1] = ellipseCoeff.c;
        ellipseMatrix.ptr<float>(0)[2] = ellipseCoeff.d / 2;
        ellipseMatrix.ptr<float>(2)[0] = ellipseCoeff.d / 2;
        ellipseMatrix.ptr<float>(1)[2] = ellipseCoeff.e / 2;
        ellipseMatrix.ptr<float>(2)[1] = ellipseCoeff.e / 2;
        return TRUE;
    }

    bool EllipseFunctionMatrix2Coeff(const cv::Mat& ellipseMatrix, EllipseCoeff& ellipseCoeff)
    {
        if (ellipseMatrix.empty())
        {
            return false;
        }
        if (ellipseMatrix.type() != CV_32FC1)
        {
            ellipseMatrix.convertTo(ellipseMatrix, CV_32FC1);
        }
        ellipseCoeff.a = ellipseMatrix.ptr<float>(0)[0];
        ellipseCoeff.b = 2 * ellipseMatrix.ptr<float>(0)[1];
        ellipseCoeff.c = ellipseMatrix.ptr<float>(1)[1];
        ellipseCoeff.d = 2 * ellipseMatrix.ptr<float>(0)[2];
        ellipseCoeff.e = 2 * ellipseMatrix.ptr<float>(1)[2];

        return TRUE;
    }
    bool FitEllipse(const std::vector<cv::Point2f>& vecEdgePt, EllipseCoeff& ellipCoeff)
    {
        if (vecEdgePt.empty())
        {
            std::cout << "用于拟合椭圆的边缘点集合为空！";
            return false;
        }
        if (vecEdgePt.size() <= 10)
        {
            std::cout << "用于拟合椭圆的边缘点太少！";
            return false;
        }
        cv::Mat_<float> matrix(vecEdgePt.size(), 5);
        cv::Mat dst(vecEdgePt.size(), 1, CV_32F, cv::Scalar(-1));
        for (size_t i = 0; i < vecEdgePt.size(); i++)
        {
            float x = vecEdgePt[i].x;
            float y = vecEdgePt[i].y;
            matrix(i, 0) = x * x;
            matrix(i, 1) = x * y;
            matrix(i, 2) = y * y;
            matrix(i, 3) = x;
            matrix(i, 4) = y;
        }
        cv::Mat ellipseParam(5, 1, CV_32F, cv::Scalar(0));
        solve(matrix, dst, ellipseParam, cv::DECOMP_SVD);
        float* ptr = ellipseParam.ptr<float>(0);
        if (ptr[1] * ptr[1] - 4 * ptr[0] * ptr[2] >= 0)//如果不是椭圆
        {
            ellipseParam.setTo(0);
        }
        ellipCoeff.a = ellipseParam.ptr<float>(0)[0];
        ellipCoeff.b = ellipseParam.ptr<float>(1)[0];
        ellipCoeff.c = ellipseParam.ptr<float>(2)[0];
        ellipCoeff.d = ellipseParam.ptr<float>(3)[0];
        ellipCoeff.e = ellipseParam.ptr<float>(4)[0];
        return true;
    }
    bool EdgeSampling(const EllipseCoeff& coeff, const int nPoints, std::vector<cv::Point2f>& vecSamplingPts, std::vector<cv::Vec3f>* vecNs)
    {
        //根据椭圆一般方程参数计算椭圆的外包围框参数
        float a = coeff.a;
        float b = coeff.b;
        float c = coeff.c;
        float d = coeff.d;
        float e = coeff.e;
        float delta = b * b - 4 * a * c;
        if (delta >= 0)
        {
            return false;
        }
        float x0 = -(b * e - 2 * c * d) / delta;
        float y0 = -(b * d - 2 * a * e) / delta;
        float r = a * x0 * x0 + b * x0 * y0 + c * y0 * y0 - 1;
        float aa = sqrt(r / a);
        float bb = sqrt(-4 * a * r / delta);
        vecSamplingPts.clear();

        //计算采样点
        for (size_t i = 0; i < nPoints + 1; i++)
        {
            float  t = i * 2 * PI / (nPoints);
            cv::Point2f point;
            point.x = aa * cos(t) - b / (2 * a) * bb * sin(t);
            point.y = bb * sin(t);
            point = point + cv::Point2f(x0, y0);
            vecSamplingPts.push_back(point);
        }
        vecSamplingPts.pop_back();

        //计算采样点法线
        if (vecNs != NULL) {
            vecNs->clear();
            for (size_t i = 0; i < nPoints; i++)
            {
                cv::Vec3f normal;
                normal[0] = 2 * c * vecSamplingPts[i].y + b * vecSamplingPts[i].x + e;
                normal[1] = -(2 * a * vecSamplingPts[i].x + b * vecSamplingPts[i].y + d);
                float temp = sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
                normal[0] = normal[0] / temp;
                normal[1] = normal[1] / temp;
                normal[2] = -normal[0] * vecSamplingPts[i].x - normal[1] * vecSamplingPts[i].y;
                vecNs->push_back(normal);
            }
        }
        return true;
    }
    bool EdgeSampling(const tlm::Point2DInfo& ptwithE, const int nPointsNum, std::vector<cv::Point2f>& vecSamplingPts, std::vector<cv::Vec3f>* vecNs)
    {
        EllipseCoeff coeff;
        if (!EllipseParam2FunctionCoeff(ptwithE, coeff))
        {
            return false;
        }
        else
        {
            EdgeSampling(coeff, nPointsNum, vecSamplingPts);
            return true;
        }
        
    }
    bool EdgeSampling(const cv::Point2f center, const cv::Size& axis, const double& angle, const int nPointsNum, std::vector<cv::Point2f>& vecSamplingPts, std::vector<cv::Vec3f>* vecNs)
    {
        EllipseCoeff coeff;
        if (!EllipseParam2FunctionCoeff(center, axis, angle, coeff))
        {
            return false;
        }
        else
        {
            EdgeSampling(coeff, nPointsNum, vecSamplingPts);
        }  
    }
    bool LineAndEllipseIntersection(const cv::Vec3f& lineCoeff, EllipseCoeff& ellipseCoeff, std::vector<cv::Point2f>& intersections)
    {
        float A = ellipseCoeff.a;
        float B = ellipseCoeff.b;
        float C = ellipseCoeff.c;
        float D = ellipseCoeff.d;
        float E = ellipseCoeff.e;
        float F = lineCoeff[0];
        float G = lineCoeff[1];
        float H = lineCoeff[2];

        double a = A * G * G / F / F - B * G / F + C;
        double b = 2 * A * G * H / F / F - B * H / F - D * G / F + E;
        double c = A * H * H / F / F - D * H / F + 1.0;
        double delta = b * b - 4.0 * a * c;
        if (delta < 0)
        {
            return false;
        }
        else
        {
            double y1 = (-b + sqrt(delta)) / 2.0 / a;
            double x1 = (-G * y1 - H) / F;
            double y2 = (-b - sqrt(delta)) / 2.0 / a;
            double x2 = (-G * y2 - H) / F;

            intersections.push_back(cv::Point2f(x1, y1));
            intersections.push_back(cv::Point2f(x2, y2));
        }
        return true;
    }

    bool LineAndEllipseIntersection(
        const double la, 
        const double lb, 
        const double lc, 
        EllipseCoeff& ellipseCoeff, 
        std::vector<cv::Point2f>& intersections)
    {
        cv::Vec3f lineCoeff(la, lb, lc);
        return LineAndEllipseIntersection(lineCoeff, ellipseCoeff, intersections);
    }

    bool LineAndEllipseIntersection(
        const cv::Vec3f& lineCoeff, 
        const cv::Point2f& center, 
        const cv::Size& axis, 
        const double& angle, 
        std::vector<cv::Point2f>& intersections)
    {
        EllipseCoeff ellipseCoeff = { center.x,center.y,axis.width, axis.height,angle };
        return LineAndEllipseIntersection(lineCoeff, ellipseCoeff, intersections);
    }

    bool LineAndEllipseIntersection(
        const double la, 
        const double lb, 
        const double lc, 
        const cv::Point2f& center, 
        const cv::Size& axis, 
        const double& angle, 
        std::vector<cv::Point2f>& intersections)
    {
        cv::Vec3f lineCoeff(la, lb, lc);
        EllipseCoeff ellipseCoeff = { center.x, center.y, axis.width, axis.height,angle };
        return LineAndEllipseIntersection(lineCoeff, ellipseCoeff, intersections);
    }

    void drawLineWithABC(cv::Mat& Image, const double a, const double b, const double c, const cv::Scalar& color)
    {
        int W = Image.cols;
        int H = Image.rows;
        cv::Point pt11(0, -c / b);
        cv::Point pt12(-c / a, 0);
        cv::Point pt21(W, -(a * W + c) / b);
        cv::Point pt22(-(b * H + c) / a, H);
        cv::Point pt1, pt2;
        if (round(-c / b) > 0)
        {
            pt1 = pt11;
        }
        else
        {
            pt1 = pt12;
        }

        if (round(-(a * W + c) / b) < H)
        {
            pt2 = pt21;
        }
        else
        {
            pt2 = pt22;
        }
        cv::line(Image, pt1, pt2, color, 1, cv::LINE_AA);
    }
}