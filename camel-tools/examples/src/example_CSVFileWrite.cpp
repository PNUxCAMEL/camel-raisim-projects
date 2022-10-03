#include <fstream>
#include <iostream>

int main(int argc, char* argv[])
{
    std::ofstream myFile;
    myFile.open("example.csv");
    myFile << "This is the first cell in the first column.\n";
    myFile << "a,b,c,";
    myFile << "c,s,v,\n";
    myFile << "1,2,3.456\n";
    myFile << "semi;colon";
    myFile.close();
    return 0;
}
