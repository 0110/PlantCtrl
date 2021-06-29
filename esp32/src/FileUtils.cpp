#include <Homie.h>
#include "FileUtils.h"

bool deleteFile(const char *source)
{
    Serial << "deleting file " << source << endl;
    if (!SPIFFS.begin())
    {
        return false;
    }
    bool deleted = SPIFFS.remove(source);
    if (deleted)
    {
        Serial << "Deleted " << source << endl;
    }
    else
    {
        Serial << "Could not delete " << source << endl;
    }
    return deleted;
}

void printFile(const char *source)
{
    Serial << "printing file " << source << endl;
    if (!SPIFFS.begin())
    {
        Serial << "could not start spiffs " << source << endl;
        return;
    }
    File file = SPIFFS.open(source, FILE_READ);
    if (!file)
    {
        Serial << "could not start open " << source << endl;
        return;
    }
    Serial << file.readString() << endl;
    Serial << "Finished printing file " << source << endl;
    file.close();
}

bool doesFileExist(const char *source)
{
    Serial << "checking if file exist " << source << endl;
    if (!SPIFFS.begin())
    {
        return false;
    }
    bool exists = SPIFFS.exists(source);
    Serial << "File " << source << (exists ? "" : " not") << " found " << endl;
    return exists;
}

bool copyFile(const char *source, const char *target)
{
    Serial << "copy started " << source << " -> " << target << endl;
    if (!SPIFFS.begin())
    {
        return false;
    }

    File file = SPIFFS.open(source, FILE_READ);
    File file2 = SPIFFS.open(target, FILE_WRITE);
    Serial.flush();
    if (!file)
    {
        Serial << "There was an error opening " << source << " for reading" << endl;
        return false;
    }
    if (!file2)
    {
        Serial << "There was an error opening " << target << " for reading" << endl;
        file.close();
        return false;
    }
    file2.println(file.readString());
    Serial << "copy finished " << source << " -> " << target << endl;
    file.close();
    file2.close();
    return true;
}