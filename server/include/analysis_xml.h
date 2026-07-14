#ifndef ANALYSIS_XML_H
#define ANALYSIS_XML_H
// 修改后的 parseXMLFile 函数
void parseXMLFile(const char* filename, int* id, double times[2], double* freqs);
void printResults(int id, double times[2], double freqs);
#endif