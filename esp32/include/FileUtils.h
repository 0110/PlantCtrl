#ifndef FILEUTILS_H
#define FILEUTILS_H

bool doesFileExist(const char *source);
bool copyFile(const char *source, const char *target);
bool deleteFile(const char *source);
void printFile(const char *source);

#endif