/*
NI device wrapper program for Python.
This code created by Eunchul Jeon, jsharp83@gmail.com
*/

#include "Python.h"
#include <stdio.h>
#include <windows.h>
#include <NIDAQmx.h>

#pragma comment(lib, "NIDAQmx.lib")

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else


char		errBuff[2048]={'\0'};
int			error=0;

const int SAMPLE_SIZE = 10;
int data1[10];
int data2[10];

// taskHandles[0 ~ 1] : handles for voltage output, taskHandles[3 ~ 4] : handles for counter
TaskHandle taskHandles[4];

static PyObject *ErrorObject;

static PyObject* initDevice(PyObject *self, PyObject *args)
{
  printf("Init NI Device\n");

  // NI Driver Initial C Source.
  
  /*********************************************/
  // DAQmx Configure Code
  /*********************************************/
  DAQmxErrChk (DAQmxCreateTask("",&taskHandles[0]));
  DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandles[0],"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,""));
  DAQmxErrChk (DAQmxCreateTask("",&taskHandles[1]));
  DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandles[1],"Dev1/ao1","",-10.0,10.0,DAQmx_Val_Volts,""));
  DAQmxErrChk (DAQmxCreateTask("",&taskHandles[2]));
  DAQmxErrChk (DAQmxCreateCICountEdgesChan(taskHandles[2],"Dev1/ctr0","",DAQmx_Val_Rising,0,DAQmx_Val_CountUp));
  DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandles[2],"/Dev1/PFI0",(double)SAMPLE_SIZE,DAQmx_Val_Rising,DAQmx_Val_ContSamps,SAMPLE_SIZE));
  DAQmxErrChk (DAQmxCreateTask("",&taskHandles[3]));
  DAQmxErrChk (DAQmxCreateCICountEdgesChan(taskHandles[3],"Dev1/ctr1","",DAQmx_Val_Rising,0,DAQmx_Val_CountUp));
  DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandles[3],"/Dev1/PFI1",(double)SAMPLE_SIZE,DAQmx_Val_Rising,DAQmx_Val_ContSamps,SAMPLE_SIZE));
  
  /*********************************************/
  // DAQmx Start Code
  /*********************************************/
  DAQmxErrChk (DAQmxStartTask(taskHandles[0]));
  DAQmxErrChk (DAQmxStartTask(taskHandles[1]));
  DAQmxErrChk (DAQmxStartTask(taskHandles[2]));
  DAQmxErrChk (DAQmxStartTask(taskHandles[3]));

  return Py_BuildValue("i", 1);

Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);

  return Py_BuildValue("i", 0);

}

static PyObject* updateVoltages(PyObject *self, PyObject *args)
{
  double volt1, volt2;
  double leftData[1];
  double rightData[1];

  if(!PyArg_ParseTuple(args, "dd", &volt1, &volt2))
    return NULL;
  
  //printf("Update Voltage values\n");

  leftData[0] = volt1;
  rightData[0] = volt2;

  /*********************************************/
  // DAQmx Write Code
  /*********************************************/
  DAQmxErrChk (DAQmxWriteAnalogF64(taskHandles[0],1,1,10.0,DAQmx_Val_GroupByChannel,leftData,NULL,NULL));
  DAQmxErrChk (DAQmxWriteAnalogF64(taskHandles[1],1,1,10.0,DAQmx_Val_GroupByChannel,rightData,NULL,NULL));

  return Py_BuildValue("i", 1);

Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);
  
  return Py_BuildValue("i", 0);
}

static PyObject* countLeftEncoderTic(PyObject *self, PyObject *args)
{

  int read = 0;


  DAQmxReadCounterU32(taskHandles[2],SAMPLE_SIZE,0,data1,SAMPLE_SIZE,&read,NULL);

  return Py_BuildValue("i", read);

Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);
  
  return Py_BuildValue("i", -1);

}

static PyObject* countRightEncoderTic(PyObject *self, PyObject *args)
{
  int read = 0;
  /* DAQmxReadCounterU32(taskHandles[3],SAMPLE_SIZE,0,data2,SAMPLE_SIZE,&read,NULL); */

  return Py_BuildValue("i", read);
  
Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);
  
  return Py_BuildValue("i", -1);
}

int leftEncoder()
{
  int read = 0;

  while(1){
    DAQmxReadCounterU32(taskHandles[2],SAMPLE_SIZE,0,data1,SAMPLE_SIZE,&read,NULL);
  }
  return read;

Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);
  
  return -1;

}

int rightEncoder(){
  int read = 0;
  DAQmxReadCounterU32(taskHandles[3],SAMPLE_SIZE,0,data2,SAMPLE_SIZE,&read,NULL); 

  return read;
  
Error:
  if( DAQmxFailed(error) )
    DAQmxGetExtendedErrorInfo(errBuff,2048);
  if( taskHandles[0]!=0 ) {
    /*********************************************/
    // DAQmx Stop Code
    /*********************************************/
    DAQmxStopTask(taskHandles[0]);
    DAQmxClearTask(taskHandles[0]);
    DAQmxStopTask(taskHandles[1]);
    DAQmxClearTask(taskHandles[1]);
    DAQmxStopTask(taskHandles[2]);
    DAQmxClearTask(taskHandles[2]);
    DAQmxStopTask(taskHandles[3]);
    DAQmxClearTask(taskHandles[3]);
    
  }
  if( DAQmxFailed(error) )
    printf("DAQmx Error: %s\n",errBuff);
  
  return -1;

}

static PyObject* readEncoder(PyObject *self, PyObject *args)
{
  int i = 0;
  int j =0 ;

  int read1 = 0;
  int read2 = 0;
  
  while(1){
    DAQmxReadCounterU32(taskHandles[2],SAMPLE_SIZE,0,data1,SAMPLE_SIZE,&read1,NULL); 
    DAQmxReadCounterU32(taskHandles[3],SAMPLE_SIZE,0,data2,SAMPLE_SIZE,&read2,NULL);

    i = i+read1;
    j = j+read2;

    read1 = 0;
    read2 = 0;

    printf("%d %d \n", i, j);
  }

}


static struct PyMethodDef methods[] = {
  {"initDevice", initDevice, METH_VARARGS},
  {"updateVoltages", updateVoltages, METH_VARARGS},
  {"countLeftEncoderTic", countLeftEncoderTic, METH_VARARGS},
  {"countRightEncoderTic", countRightEncoderTic, METH_VARARGS},
  {"readEncoder", readEncoder, METH_VARARGS},
  {NULL, NULL}
};

void initEncoderReader()
{
  PyObject* m;

  m = Py_InitModule("EncoderReader", methods);
  ErrorObject = Py_BuildValue("s", "error");
}
