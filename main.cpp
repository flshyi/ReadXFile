#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>
#include <chrono>
#include <list>
#include<iomanip>
#include "tinyxml2.h"
#include "read_device_calibration_xml.h"

#include <Eigen/Core>

using namespace std;
using namespace tinyxml2;

   void stringsToInt(const char* input_string, std::vector<int> &out_num)
    {
        if (nullptr == input_string)
        {
            return;
        }
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
        if (nullptr == input_string)
        {
            return;
        }
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




int main(int argc, char **argv) {

    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: error just " << argc <<"params." << std::endl;
        return 1;
    }
    CalibrationResult calibResult;
            bool bOK = calibResult.LoadXML(argv[1]);
 
    return 0;
}



