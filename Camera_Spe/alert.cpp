    /**g++ -o callpy callpy.cpp -I/usr/include/python2.6 -L/usr/lib64/python2.6/config -lpython2.6**/  
    #include <python2.7/Python.h>  
    #include <iostream>  
    int main(int argc, char** argv)  
    {  
        
        Py_Initialize();  
      
      
        if ( !Py_IsInitialized() ) {  
            return -1;  
        }  
       
        PyRun_SimpleString("import sys");  
        PyRun_SimpleString("print '---import sys---'");   
        PyRun_SimpleString("sys.path.append('./')");  
        PyObject *pName,*pModule,*pDict,*pFunc,*pArgs;  
      
      
        pName = PyString_FromString("alert");  
        pModule = PyImport_Import(pName);  
        if ( !pModule ) {  
            printf("can't find alert.py");  
            getchar();  
            return -1;  
        }  
        pDict = PyModule_GetDict(pModule);  
        if ( !pDict ) {  
            return -1;  
        }  
      
      
        printf("----------------------\n");  
        pFunc = PyDict_GetItemString(pDict, "alertMe");  
        if ( !pFunc || !PyCallable_Check(pFunc) ) {  
            printf("can't find function [alertMe]");  
            getchar();  
            return -1;  
         }  
      
       
      
        PyObject_CallObject(pFunc, NULL);  
      
      
        Py_DECREF(pName);  
        Py_DECREF(pModule);  
      
     
        Py_Finalize();  
        return 0;  
    }   
