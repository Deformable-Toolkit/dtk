#include<iostream>
using namespace std;


#include "dtk.h"
#include "dtkErrorManager.h"
#include "dtkErrorManager.cpp"
#include "dtkError.h"

using namespace dtk;
int main(){
    //cout << &dtk::dtkErrMgr << endl;
    //cout << dtk::dtkErrMgr.GetErrorString(dtk::dtkError::OUT_OF_RANGE) << endl;
    cout << &dtk::dtkErrorManager::GetInstance() << endl;
    cout << &dtk::dtkErrorManager::GetInstance() << endl;
    return 0;
}