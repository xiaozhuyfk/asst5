#include "CameraSensor.h"
#include "Image.h"

class CameraPipeline {
private:
    CameraSensor* sensor;
    int initialFocus;

private:

    void AutoFocus() {
        sensor->SetFocus(initialFocus);
    }

    int in_bound(int w, int h, int i, int j) {
        if (i < 0 || i >= h || j < 0 || j >= w) {
            return 0;
        }
        return 1;
    }

    unsigned char demosiac_adj(unsigned char *buf, int w, int h, int i, int j) {
        if (!in_bound(w, h, i, j)) {
            return 0;
        }

        unsigned char res = 0;
        int r0 = i - 1; int c0 = j;
        int r1 = i; int c1 = j - 1;
        int r2 = i; int c2 = j + 1;
        int r3 = i + 1; int c3 = j;
        int count = in_bound(w, h, r0, c0) +
                in_bound(w, h, r1, c1) +
                in_bound(w, h, r2, c2) +
                in_bound(w, h, r3, c3);

        if (in_bound(w, h, r0, c0)) {
            res += buf[r0 * w + c0] / count;
        }

        if (in_bound(w, h, r1, c1)) {
            res += buf[r1 * w + c1] / count;
        }

        if (in_bound(w, h, r2, c2)) {
            res += buf[r2 * w + c2] / count;
        }

        if (in_bound(w, h, r3, c3)) {
            res += buf[r3 * w + c3] / count;
        }

        return res;

    }

    unsigned char demosiac_diagnal(unsigned char *buf, int w, int h, int i, int j) {
        if (!in_bound(w, h, i, j)) {
            return 0;
        }

        unsigned char res = 0;
        int r0 = i - 1; int c0 = j - 1;
        int r1 = i - 1; int c1 = j + 1;
        int r2 = i + 1; int c2 = j - 1;
        int r3 = i + 1; int c3 = j + 1;

        int count = in_bound(w, h, r0, c0) +
                in_bound(w, h, r1, c1) +
                in_bound(w, h, r2, c2) +
                in_bound(w, h, r3, c3);

        if (in_bound(w, h, r0, c0)) {
            res += buf[r0 * w + c0] / count;
        }

        if (in_bound(w, h, r1, c1)) {
            res += buf[r1 * w + c1] / count;
        }

        if (in_bound(w, h, r2, c2)) {
            res += buf[r2 * w + c2] / count;
        }

        if (in_bound(w, h, r3, c3)) {
            res += buf[r3 * w + c3] / count;
        }

        return res;
    }

    unsigned char demosiac_vertical(unsigned char *buf, int w, int h, int i, int j) {
        if (!in_bound(w, h, i, j)) {
            return 0;
        }

        unsigned char res = 0;
        int r0 = i - 1; int c0 = j;
        int r1 = i + 1; int c1 = j;
        int count = in_bound(w, h, r0, c0) + in_bound(w, h, r1, c1);

        if (in_bound(w, h, r0, c0)) {
            res += buf[r0 * w + c0] / count;
        }

        if (in_bound(w, h, r1, c1)) {
            res += buf[r1 * w + c1] / count;
        }

        return res;
    }

    unsigned char demosiac_horizontal(unsigned char *buf, int w, int h, int i, int j) {
        if (!in_bound(w, h, i, j)) {
            return 0;
        }

        unsigned char res = 0;
        int r0 = i; int c0 = j - 1;
        int r1 = i; int c1 = j + 1;
        int count = in_bound(w, h, r0, c0) + in_bound(w, h, r1, c1);

        if (in_bound(w, h, r0, c0)) {
            res += buf[r0 * w + c0] / count;
        }

        if (in_bound(w, h, r1, c1)) {
            res += buf[r1 * w + c1] / count;
        }

        return res / count;
    }

    void ProcessShot(Image & result, unsigned char * inputBuffer, int w,
            int h) {
        // the current implementation just replicates the pixel's
        // measurement into the RGB channels of the output image
        auto pix = inputBuffer;
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                auto & p = result.Pixels[i * w + j];
                //p.r = p.b = p.g = pix[i * w + j];
                if ((i + j) % 2 == 0) {
                    // green pixel
                    p.g = pix[i * w + j];
                    p.b = demosiac_horizontal(inputBuffer, w, h, i, j);
                    p.r = demosiac_vertical(inputBuffer, w, h, i, j);
                } else if (i % 2 == 0) {
                    // red pixel
                    p.r = pix[i * w + j];
                    p.g = demosiac_adj(inputBuffer, w, h, i, j);
                    p.b = demosiac_diagnal(inputBuffer, w, h, i, j);
                } else if (i % 2 == 1) {
                    // blue pixel
                    p.b = pix[i * w + j];
                    p.g = demosiac_adj(inputBuffer, w, h, i, j);
                    p.r = demosiac_diagnal(inputBuffer, w, h, i, j);
                }
            }
        }
    }

public:

    CameraPipeline(CameraSensor * sensor, int initialFocus) {
        this->sensor = sensor;
        this->initialFocus = initialFocus;
    }

    // Obtain pixel values from sensor, process the sensor output, and
    // then stick RGB values into the pixels of 'result'
    void TakePicture(Image & result) {
        result.Width = sensor->GetImageWidth();
        result.Height = sensor->GetImageHeight();
        result.Pixels.SetSize(result.Width * result.Height);

        // buffer for storing bits read from sensor
        List<unsigned char> buffer;
        buffer.SetSize(result.Width * result.Height);

        // focus the camera
        AutoFocus();

        // take a shot (arguments to ReadSensorData() define a
        // cropwindow of the sensor: left, top, width, height)
        //
        // In the example below, we grab the whole sensor
        sensor->ReadSensorData(buffer.Buffer(), 0, 0, result.Width,
                result.Height);

        // process the shot
        ProcessShot(result, buffer.Buffer(), result.Width, result.Height);
    }
};

void PrintUsage(char* programName) {
    printf("usage: %s [options] inputfile outputfile\n", programName);
    printf("  Options:\n");
    printf("     -focus INT            Set default focus\n");
    printf(
            "     -noiselevel [1-4]     Set amount of sensor measurement noise (default=1)\n");
    printf("     -verybadsensor        Turn on low-quality sensor mode\n");
    printf("     -help                 This message\n");
    printf("\n");
}

int main(int argc, char* argv[]) {

    int noiseLevel = 1;
    bool badSensor = false;
    int initialFocus = 650;

    int optind = 1;
    int i;
    for (i = 1; i < argc; i++) {
        if (String(argv[i]) == L"-noiselevel" && i + 1 < argc) {
            noiseLevel = Math::Clamp(StringToInt(String(argv[i + 1])), 0, 4);
            i++;
        } else if (String(argv[i]) == L"-focus") {
            initialFocus = StringToInt(String(argv[i + 1]));
            i++;
        } else if (String(argv[i]) == L"-verybadsensor")
            badSensor = true;

        else if (String(argv[i]) == L"-help") {
            PrintUsage(argv[0]);
            return 0;
        } else
            break;

    }
    optind = i;

    if (optind + 1 >= argc) {
        PrintUsage(argv[0]);
        return 0;
    }

    String inputFilename = argv[optind];
    String outputFilename = argv[optind + 1];

    RefPtr < CameraSensor > sensor = CreateSensor(inputFilename.Buffer(),
            noiseLevel, badSensor);
    {
        CameraPipeline pipe(sensor.Ptr(), initialFocus);
        Image result;
        pipe.TakePicture(result);
        result.Save(outputFilename);
    }

    return 0;
}
