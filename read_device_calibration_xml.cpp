#include "read_device_calibration_xml.h"
#include "tinyxml2.h"
#include <iostream>

using namespace std;
using namespace tinyxml2;

/***************************
Example1: new device_calibration.xml

<?xml version="1.0" encoding="UTF-8" ?>
<DeviceConfiguration deviceUID="183095663" cameraModel="ov9282">
<Camera id="0" name="left">
<Calibration size="640 400 " principal_point="314.17169 192.01466 " focal_length="282.64291 282.64291 " fish_eye="true" radial_distortion_8="-0.009060638 0.011490704 0.00057481864 -0.0017591408 0 0 0 0 " model="FISHEYE_4_PARAMETERS" radial_distortion="-0.009060638 0.011490704 0.00057481864 -0.0017591408 0 0 " />
<Rig translation="0 0 0 " rowMajorRotationMat="1 0 0 0 1 0 0 0 1 " />
</Camera>
<Camera id="1" name="right">
<Calibration size="640 400 " principal_point="313.89957 197.20532 " focal_length="283.63623 283.63623 " fish_eye="true" radial_distortion_8="-0.0096135745 0.011664068 -0.00034470914 -0.0014093876 0 0 0 0 " model="FISHEYE_4_PARAMETERS" radial_distortion="-0.0096135745 0.011664068 -0.00034470914 -0.0014093876 0 0 " />
<Rig translation="-0.079340182 -0.00013975039 -0.00040562815 " rowMajorRotationMat="0.99999362 -0.0016118266 -0.0031758258 0.0016010099 0.99999291 -0.0034055756 0.0031812927 0.0034004694 0.99998915 " />
</Camera>
<Image format="interleaved" />
<SFConfig>
<Stateinit ombc="2.2313454 -2.2157061 -0.00027956281 " tbc="0.0014527332 0.011554177 0.00059698889 " aBias="0.0091352137 0.0044898326 -0.12521665 " wBias="-0.00022343617 2.1904672e-05 -0.00019966695 " ka="-0.0035309654 0.0018503405 -0.0033280752 " kg="-0.0025925508 -0.0028574141 0.0035696717 " delta="0.001739" />
</SFConfig>
</DeviceConfiguration>

Example2: origin device_calibration.xml

<?xml version="1.0" encoding="UTF-8" ?>
<DeviceConfiguration deviceUID="2980551625" cameraModel="ov9282">
<Camera id="0" name="left">
<Calibration size="640 400 " principal_point="321.52335 200.5928 " focal_length="285.17496 285.17496 " fish_eye="true" radial_distortion_8="-0.0037936682 0.00019264412 0.0084389299 -0.0038834778 0 0 0 0 " />
<Rig stereo_rig="0 0 0 0 0 0 " />
</Camera>
<Camera id="1" name="right">
<Calibration size="640 400 " principal_point="323.04761 204.19971 " focal_length="285.43146 285.43146 " fish_eye="true" radial_distortion_8="-0.0051394142 0.0052400897 0.0028621815 -0.0020638788 0 0 0 0 " />
<Rig stereo_rig="-0.077307917 0.00023512765 -0.0011964927 -0.0040601059 0.0054506841 0.0011773829 " />
</Camera>
<Image format="interleaved" />
<SFConfig>
<Stateinit ombc="2.2234266 -2.21645 -0.0043120459 " tbc="0.00046945253 0.011790942 -0.0036639881 " aBias="-0.16761935 -0.00299799 -0.14357091 " wBias="-2.7723701e-05 -0.00098237919 0.00021645613 " ka="-0.0031317824 0.006244815 -0.0044185193 " kg="-0.0053632134 -0.0047300262 0.006965823 " delta="0.001913" />
</SFConfig>
</DeviceConfiguration>

****************************/

CameraPara::CameraPara()
{
    useFisheyeMode = true;
    fx = 282.64291;
    fy = 282.64291;
    cx = 314.17169;
    cy = 192.01466;
    cameraMatrix = (cv::Mat_<double>(3,3)<< fx, 0, cx, 0, fy , cy, 0, 0, 1);
    distortCoeffs = (cv::Mat_<double>(4, 1) << -0.009060638, 0.011490704, 0.00057481864, - 0.0017591408);
    imageSize = cv::Size(640, 400);
    w = 0.0;
    R = cv::Mat::zeros(3, 1, CV_64FC1);
    T = cv::Mat::zeros(3, 1, CV_64FC1);
}

CalibrationResult::CalibrationResult()
{
    right.fx = 283.63623;
    right.fy = 283.63623;
    right.cx = 313.89957;
    right.cy = 197.20532;
    right.cameraMatrix = (cv::Mat_<double>(3, 3) << right.fx, 0, right.cx, 0, right.fy, right.cy, 0, 0, 1);
    right.distortCoeffs = (cv::Mat_<double>(4, 1) << -0.0096135745, 0.011664068, - 0.00034470914, - 0.0014093876);
    //right.R = (cv::Mat_<double>(3, 3) << 0.99999362, -0.0016118266, -0.0031758258,
    //									 0.0016010099, 0.99999291, -0.0034055756,
    //									 0.0031812927, 0.0034004694, 0.99998915);
    right.R = (cv::Mat_<double>(3, 1) << 0.003403036291820421, -0.00317857217762142, 0.001606424784838596);
    right.T = (cv::Mat_<double>(3, 1) << -0.079340182, - 0.00013975039, - 0.00040562815);

    E = cv::Mat::eye(3, 3, CV_64FC1);
    F = cv::Mat::eye(3, 3, CV_64FC1);

    double ombc[3] = { 2.2313454, - 2.2157061, - 0.00027956281 };
    double tbc[3] = { 0.0014527332, 0.011554177, 0.0005969888 };
    double aBias[3] = { 0.0091352137, 0.0044898326, - 0.12521665 };
    double wBias[3] = { -0.00022343617, 2.1904672e-05, - 0.00019966695 };
    double ka[3] = { -0.0035309654, 0.0018503405, - 0.0033280752 };
    double kg[3] = { -0.0025925508, - 0.0028574141, 0.0035696717 };
    //vec_ombc = std::vector<double>(ombc,ombc+3);
    vec_ombc << 2.2313454, - 2.2157061, - 0.00027956281;
    vec_tbc = std::vector<double>(tbc,tbc+3);
    vec_aBias = std::vector<double>(aBias,aBias+3);
    vec_wBias = std::vector<double>(wBias,wBias+3);
    vec_ka = std::vector<double>(ka,ka+3);
    vec_kg = std::vector<double>(kg,kg+3);
    dDelta = 0.00173;

}

void stringsToInt(const char* input_string, std::vector<int> &out_num)
{
    if (NULL == input_string)
    {
        return;
    }
    out_num.clear();
    std::string sInput(input_string);
    int length = static_cast<int>(sInput.length());
    const char cBlank = ' ';
    std::vector<int> vecBlankPos;
    std::vector<string> vecStrings;
    int iLastPos = 0;
    for (int i = 0; i < length; ++i)
    {
        if (cBlank == sInput[i])
        {
            std::string tmp = sInput.substr(iLastPos, i - iLastPos);
            vecStrings.push_back(tmp);

            std::stringstream ss(tmp);
            int iValue;
            ss >> iValue;
            out_num.push_back(iValue);

            vecBlankPos.push_back(i);
            iLastPos = i;
        }
    }
}

void stringsToDouble(const char* input_string, std::vector<double> &out_num)
{
    if (NULL == input_string)
    {
        return;
    }
    out_num.clear();
    std::string sInput(input_string);
    size_t length = sInput.length();
    const char cBlank = ' ';
    std::vector<int> vecBlankPos;
    std::vector<string> vecStrings;
    int iLastPos = 0;
    for (int i = 0; i < length; ++i)
    {
        if (cBlank == sInput[i])
        {
            std::string tmp = sInput.substr(iLastPos, i - iLastPos);
            vecStrings.push_back(tmp);

            std::stringstream ss(tmp);
            double dValue;
            ss >> dValue;
            out_num.push_back(dValue);

            vecBlankPos.push_back(i);
            iLastPos = i;
        }
    }
}

bool CalibrationResult::LoadXML(const char* file_name)
{
    if (NULL == file_name)
    {
        return false;
    }

    XMLDocument docXml;
    XMLError errXml = docXml.LoadFile(file_name);
    if (XML_SUCCESS == errXml)
    {
        XMLElement* elmtRoot = docXml.RootElement();
        if (NULL == elmtRoot)
        {
            return false;
        }
        //int iDeviceUID = elmtRoot->IntAttribute("deviceUID");
        //const char* pCameraModel = elmtRoot->Attribute("cameraModel");

        bool bLoadLeftOk = false, bLoadRightOK = false, bLoadSFConfigOK = false;

        ////read new device_calibration
        XMLElement *elmtCamera = elmtRoot->FirstChildElement("Camera");
        if (elmtCamera)
        {
            int iID = elmtCamera->IntAttribute("id");
            const char* cName = elmtCamera->Attribute("name");

            XMLElement* pElementCalib = elmtCamera->FirstChildElement("Calibration");
            const char* pSize = pElementCalib->Attribute("size");
            const char* pCxy = pElementCalib->Attribute("principal_point");
            const char* pFxy = pElementCalib->Attribute("focal_length");
            const char* pDistort = pElementCalib->Attribute("radial_distortion_8");
            bool bFisheye = pElementCalib->BoolAttribute("fish_eye");
            const char* pModel = pElementCalib->Attribute("model");
            const char* pRadDistort = pElementCalib->Attribute("radial_distortion");

            XMLElement* pElementRig = elmtCamera->FirstChildElement("Rig");
            const char* pTranslation = pElementRig->Attribute("translation");
            const char* pRotation = pElementRig->Attribute("rowMajorRotationMat");

            std::vector<int> vecSize;
            std::vector<double> vecCxy, vecFxy, vecDistort, vecT, vecR;
            stringsToInt(pSize, vecSize);
            stringsToDouble(pCxy, vecCxy);
            stringsToDouble(pFxy, vecFxy);
            stringsToDouble(pDistort, vecDistort);
            stringsToDouble(pTranslation, vecT);
            stringsToDouble(pRotation, vecR);

            if (vecSize.size() == 2 && vecCxy.size() == 2 && vecFxy.size() == 2 &&
                vecDistort.size() >= 4 && vecT.size() == 3 && vecR.size() == 9)
            {
                bLoadLeftOk = true;
                left.useFisheyeMode = bFisheye;
                left.fx = vecFxy[0];
                left.fy = vecFxy[1];
                left.cx = vecCxy[0];
                left.cy = vecCxy[1];
                left.imageSize = cv::Size(vecSize[0], vecSize[1]);
                left.cameraMatrix = (cv::Mat_<double>(3, 3) << vecFxy[0], 0, vecCxy[0], 0, vecFxy[1], vecCxy[1], 0, 0, 1);
                if (bFisheye)
                {
                    left.distortCoeffs = (cv::Mat_<double>(4, 1) << vecDistort[0], vecDistort[1], vecDistort[2], vecDistort[3]);
                }
                else
                {
                    left.w = vecDistort[0];
                }

                left.T = (cv::Mat_<double>(3, 1) << vecT[0], vecT[1], vecT[2]);
                cv::Mat mR = (cv::Mat_<double>(3, 3) << vecR[0], vecR[1], vecR[2], vecR[3], vecR[4], vecR[5], vecR[6], vecR[7], vecR[8]);
                cv::Rodrigues(mR, left.R);
            }
        }

        XMLElement *elmtCamera2 = elmtCamera->NextSiblingElement("Camera");
        if (elmtCamera2)
        {
            int iID = elmtCamera2->IntAttribute("id");
            const char* cName = elmtCamera2->Attribute("name");

            XMLElement* pElementCalib = elmtCamera2->FirstChildElement("Calibration");
            const char* pSize = pElementCalib->Attribute("size");
            const char* pCxy = pElementCalib->Attribute("principal_point");
            const char* pFxy = pElementCalib->Attribute("focal_length");
            const char* pDistort = pElementCalib->Attribute("radial_distortion_8");
            bool bFisheye = pElementCalib->BoolAttribute("fish_eye");
            const char* pModel = pElementCalib->Attribute("model");
            const char* pRadDistort = pElementCalib->Attribute("radial_distortion");

            XMLElement* pElementRig = elmtCamera2->FirstChildElement("Rig");
            const char* pTranslation = pElementRig->Attribute("translation");
            const char* pRotation = pElementRig->Attribute("rowMajorRotationMat");

            std::vector<int> vecSize;
            std::vector<double> vecCxy, vecFxy, vecDistort, vecT, vecR;
            stringsToInt(pSize, vecSize);
            stringsToDouble(pCxy, vecCxy);
            stringsToDouble(pFxy, vecFxy);
            stringsToDouble(pDistort, vecDistort);
            stringsToDouble(pTranslation, vecT);
            stringsToDouble(pRotation, vecR);

            if (vecSize.size() == 2 && vecCxy.size() == 2 && vecFxy.size() == 2 &&
                vecDistort.size() >= 4 && vecT.size() == 3 && vecR.size() == 9)
            {
                bLoadRightOK = true;
                right.useFisheyeMode = bFisheye;
                right.fx = vecFxy[0];
                right.fy = vecFxy[1];
                right.cx = vecCxy[0];
                right.cy = vecCxy[1];
                right.imageSize = cv::Size(vecSize[0], vecSize[1]);
                right.cameraMatrix = (cv::Mat_<double>(3, 3) << vecFxy[0], 0, vecCxy[0], 0, vecFxy[1], vecCxy[1], 0, 0, 1);
                if (bFisheye)
                {
                    right.distortCoeffs = (cv::Mat_<double>(4, 1) << vecDistort[0], vecDistort[1], vecDistort[2], vecDistort[3]);
                }
                else
                {
                    right.w = vecDistort[0];
                }

                right.T = (cv::Mat_<double>(3, 1) << vecT[0], vecT[1], vecT[2]);
                cv::Mat mR = (cv::Mat_<double>(3, 3) << vecR[0], vecR[1], vecR[2], vecR[3], vecR[4], vecR[5], vecR[6], vecR[7], vecR[8]);
                cv::Rodrigues(mR, right.R);
            }
        }

        XMLElement *elmtCamera3 = elmtCamera->NextSiblingElement("SFConfig");
        if (elmtCamera3)
        {
            XMLElement* pElementCalib = elmtCamera3->FirstChildElement("Stateinit");
            const char* pombc = pElementCalib->Attribute("ombc");
            const char* ptbc = pElementCalib->Attribute("tbc");
            const char* paBias = pElementCalib->Attribute("aBias");
            const char* pwBias = pElementCalib->Attribute("wBias");
            const char* pka = pElementCalib->Attribute("ka");
            const char* pkg = pElementCalib->Attribute("kg");
            dDelta = pElementCalib->DoubleAttribute("delta");
            stringsToDouble(pombc, vec_ombc);
            stringsToDouble(ptbc, vec_tbc);
            stringsToDouble(paBias, vec_aBias);
            stringsToDouble(pwBias, vec_wBias);
            stringsToDouble(pka, vec_ka);
            stringsToDouble(pkg, vec_kg);

            for(int i = 0; i < vec_ombc.size(); i++)
                std::cout << vec_ombc[i] << ", ";
            std::cout << std::endl;
            for(int i = 0; i < vec_tbc.size(); i++)
                std::cout << vec_tbc[i] << ", ";
            std::cout << std::endl;
            for(int i = 0; i < vec_aBias.size(); i++)
                std::cout << vec_aBias[i] << ", ";
            std::cout << std::endl;
            for(int i = 0; i < vec_wBias.size(); i++)
                std::cout << vec_wBias[i] << ", ";
            std::cout << std::endl;
            for(int i = 0; i < vec_ka.size(); i++)
                std::cout << vec_ka[i] << ", ";
            std::cout << std::endl;
            for(int i = 0; i < vec_kg.size(); i++)
                std::cout << vec_kg[i] << ", ";
            std::cout << std::endl;


            std::cout << vec_ombc.size() <<"," <<
                         vec_tbc.size() <<"," <<
                         vec_aBias.size() <<"," <<
                         vec_wBias.size() <<"," <<
                         vec_ka.size() <<"," <<
                         vec_kg.size() <<"," <<std::endl;

            std::cout << std::endl;

            if (vec_ombc.size() == 3 && vec_tbc.size() == 3 && vec_aBias.size() == 3 &&
                vec_wBias.size() == 3 && vec_ka.size() == 3 && vec_kg.size() == 3)
            {
                bLoadSFConfigOK = true;
            }
        }

        bool bGetParametersOK = bLoadLeftOk && bLoadRightOK && bLoadSFConfigOK;

        if (!bGetParametersOK) //// read new failed , try to read old device-calibration.xml
        {
            XMLElement *elmtCamera = elmtRoot->FirstChildElement("Camera");
            if (elmtCamera)
            {
                int iID = elmtCamera->IntAttribute("id");
                const char* cName = elmtCamera->Attribute("name");

                XMLElement* pElementCalib = elmtCamera->FirstChildElement("Calibration");
                const char* pSize = pElementCalib->Attribute("size");
                const char* pCxy = pElementCalib->Attribute("principal_point");
                const char* pFxy = pElementCalib->Attribute("focal_length");
                const char* pDistort = pElementCalib->Attribute("radial_distortion_8");
                bool bFisheye = pElementCalib->BoolAttribute("fish_eye");

                XMLElement* pElementRig = elmtCamera->FirstChildElement("Rig");
                const char* pRig = pElementRig->Attribute("stereo_rig");

                std::vector<int> vecSize;
                std::vector<double> vecCxy, vecFxy, vecDistort, vecRig;
                stringsToInt(pSize, vecSize);
                stringsToDouble(pCxy, vecCxy);
                stringsToDouble(pFxy, vecFxy);
                stringsToDouble(pDistort, vecDistort);
                stringsToDouble(pRig, vecRig);

                if (vecSize.size() == 2 && vecCxy.size() == 2 && vecFxy.size() == 2 &&
                    vecDistort.size() == 8 && vecRig.size() == 6)
                {
                    bLoadLeftOk = true;
                    left.useFisheyeMode = bFisheye;
                    left.fx = vecFxy[0];
                    left.fy = vecFxy[1];
                    left.cx = vecCxy[0];
                    left.cy = vecCxy[1];
                    left.imageSize = cv::Size(vecSize[0], vecSize[1]);
                    left.cameraMatrix = (cv::Mat_<double>(3, 3) << vecFxy[0], 0, vecCxy[0], 0, vecFxy[1], vecCxy[1], 0, 0, 1);
                    if (bFisheye)
                    {
                        left.distortCoeffs = (cv::Mat_<double>(4, 1) << vecDistort[0], vecDistort[1], vecDistort[2], vecDistort[3]);
                    }
                    else
                    {
                        left.w = vecDistort[0];
                    }

                    left.T = (cv::Mat_<double>(3, 1) << vecRig[0], vecRig[1], vecRig[2]);
                    left.R = (cv::Mat_<double>(3, 1) << vecRig[3], vecRig[4], vecRig[5]);
                }
            }

            XMLElement *elmtCamera2 = elmtCamera->NextSiblingElement("Camera");
            if (elmtCamera2)
            {
                int iID = elmtCamera2->IntAttribute("id");
                const char* cName = elmtCamera2->Attribute("name");

                XMLElement* pElementCalib = elmtCamera2->FirstChildElement("Calibration");
                const char* pSize = pElementCalib->Attribute("size");
                const char* pCxy = pElementCalib->Attribute("principal_point");
                const char* pFxy = pElementCalib->Attribute("focal_length");
                const char* pDistort = pElementCalib->Attribute("radial_distortion_8");
                bool bFisheye = pElementCalib->BoolAttribute("fish_eye");

                XMLElement* pElementRig = elmtCamera2->FirstChildElement("Rig");
                const char* pRig = pElementRig->Attribute("stereo_rig");

                std::vector<int> vecSize;
                std::vector<double> vecCxy, vecFxy, vecDistort, vecRig;
                stringsToInt(pSize, vecSize);
                stringsToDouble(pCxy, vecCxy);
                stringsToDouble(pFxy, vecFxy);
                stringsToDouble(pDistort, vecDistort);
                stringsToDouble(pRig, vecRig);

                if (vecSize.size() == 2 && vecCxy.size() == 2 && vecFxy.size() == 2 &&
                    vecDistort.size() == 8 && vecRig.size() == 6)
                {
                    bLoadRightOK = true;
                    right.useFisheyeMode = bFisheye;
                    right.fx = vecFxy[0];
                    right.fy = vecFxy[1];
                    right.cx = vecCxy[0];
                    right.cy = vecCxy[1];
                    right.imageSize = cv::Size(vecSize[0], vecSize[1]);
                    right.cameraMatrix = (cv::Mat_<double>(3, 3) << vecFxy[0], 0, vecCxy[0], 0, vecFxy[1], vecCxy[1], 0, 0, 1);
                    if (bFisheye)
                    {
                        right.distortCoeffs = (cv::Mat_<double>(4, 1) << vecDistort[0], vecDistort[1], vecDistort[2], vecDistort[3]);
                    }
                    else
                    {
                        right.w = vecDistort[0];
                    }

                    right.T = (cv::Mat_<double>(3, 1) << vecRig[0], vecRig[1], vecRig[2]);
                    right.R = (cv::Mat_<double>(3, 1) << vecRig[3], vecRig[4], vecRig[5]);
                }
            }

            XMLElement *elmtCamera3 = elmtCamera->NextSiblingElement("SFConfig");
            if (elmtCamera3)
            {
                XMLElement* pElementCalib = elmtCamera3->FirstChildElement("Stateinit");
                const char* pombc = pElementCalib->Attribute("ombc");
                const char* ptbc = pElementCalib->Attribute("tbc");
                const char* paBias = pElementCalib->Attribute("aBias");
                const char* pwBias = pElementCalib->Attribute("wBias");
                const char* pka = pElementCalib->Attribute("ka");
                const char* pkg = pElementCalib->Attribute("kg");
                dDelta = pElementCalib->DoubleAttribute("delta");
                stringsToDouble(pombc, vec_ombc);
                stringsToDouble(ptbc, vec_tbc);
                stringsToDouble(paBias, vec_aBias);
                stringsToDouble(pwBias, vec_wBias);
                stringsToDouble(pka, vec_ka);
                stringsToDouble(pkg, vec_kg);

                if (vec_ombc.size() == 3 && vec_tbc.size() == 3 && vec_aBias.size() == 3 &&
                    vec_wBias.size() == 3 && vec_ka.size() == 3 && vec_kg.size() == 3)
                {
                    bLoadSFConfigOK = true;
                }
            }

            bGetParametersOK = bLoadLeftOk && bLoadRightOK && bLoadSFConfigOK;
        }

        if (bGetParametersOK)
        {
            /*double t[3] = { right.T.at<double>(0), right.T.at<double>(1), right.T.at<double>(2) };
            cv::Mat mTINV= (cv::Mat_<double>(3,3)<< 0, -t[2], t[1], t[2], 0, -t[0], -t[1], t[0], 0);
            cv::Mat mR;
            if (right.R.total() == 3)
            {
            cv::Rodrigues(right.R, mR);
            }
            else
            {
            right.R.copyTo(mR);
            }
            E = mTINV*mR;
            F = (right.cameraMatrix.inv()).t()*E*(left.cameraMatrix.inv());*/

            return (left.useFisheyeMode == right.useFisheyeMode);
        }
        else
        {
            return false;
        }
    }
    return false;
}
