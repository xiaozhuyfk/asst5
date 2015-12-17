#include "CameraSensor.h"
#include "Image.h"

#include <stdio.h>

#define BLUR_RANGE 3
#define ROW_BLUR_RANGE 1
#define ROW_BLUR_RANGE_LEVEL_2 5
#define G_COMP 0.95
#define PIX_THRESHOLD 40
#define ROW_THRESHOLD 1
#define PRE_THRESHOLD 40
#define BRIGHTNESS_GAIN 1.05

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

    unsigned char demosiac_extended_cross(unsigned char *buf, int w, int h, int i, int j) {
        if (!in_bound(w, h, i, j)) {
            return 0;
        }

        unsigned char res = 0;
        int r0 = i - 2; int c0 = j;
        int r1 = i; int c1 = j - 2;
        int r2 = i; int c2 = j + 2;
        int r3 = i + 2; int c3 = j;
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

    void noise_removal(Image &result, int w, int h) {
        // Noise removal
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                auto & p = result.Pixels[i * w + j];
                // Find bad points accross all three channels
                unsigned int avr;
                unsigned int avg;
                unsigned int avb;

                int count = 0;
                for (int di = -BLUR_RANGE; di <= BLUR_RANGE; di++) {
                    for (int dj = -BLUR_RANGE; dj <= BLUR_RANGE; dj++) {
                        if ((di != 0) && (dj != 0) &&
                            (((i+di) * w + j + dj) >= 0) &&
                            (((i+di) * w + j + dj) < w*h)) {
                            count++;
                            avr += result.Pixels[(i+di) * w + j + dj].r;
                            avg += result.Pixels[(i+di) * w + j + dj].g;
                            avb += result.Pixels[(i+di) * w + j + dj].b;
                        }
                    }
                }
                avr /= count;
                avg /= count;
                avb /= count;

                if (abs(p.r - avr) > PIX_THRESHOLD) {
                    p.r = (unsigned char)avr;
                }
                if (abs(p.g - avg) > PIX_THRESHOLD) {
                    p.g = (unsigned char)avg;
                }
                if (abs(p.b - avb) > PIX_THRESHOLD) {
                    p.b = (unsigned char)avb;
                }

            }
        }
    }

    unsigned long long RowAverageGreen(Image &result, int w, int row) {
        unsigned long long avg;
        for (int j = 0; j < w; j++) {
            auto &p = result.Pixels[row * w + j];
            avg += p.g;
        }

        avg /= (unsigned long long)w;
        return (unsigned char) avg;
    }

    unsigned long long RowAverageRed(Image &result, int w, int row) {
        unsigned long long avg;
        for (int j = 0; j < w; j++) {
            auto &p = result.Pixels[row * w + j];
            avg += p.r;
        }

        avg /= (unsigned long long)w;
        return (unsigned char) avg;
    }

    unsigned long long RowAverageBlue(Image &result, int w, int row) {
        unsigned long long avg;
        for (int j = 0; j < w; j++) {
            auto &p = result.Pixels[row * w + j];
            avg += p.b;
        }

        avg /= (unsigned long long)w;
        return (unsigned char) avg;
    }

    void remove_stipes(Image &result, int w, int h, int range) {

        unsigned long long avg_r = 0;
        unsigned long long avg_g = 0;
        unsigned long long avg_b = 0;
        unsigned long long cur_r = 0;
        unsigned long long cur_g = 0;
        unsigned long long cur_b = 0;
        for (int r = 0; r < h; r++) {
            avg_r = 0;
            avg_g = 0;
            avg_b = 0;

            cur_r = RowAverageRed(result, w, r);
            cur_g = RowAverageGreen(result, w, r);
            cur_b = RowAverageBlue(result, w, r);
            unsigned int count = 0;
            for (int dr = -range; dr <= range; dr++) {

                if ((r + dr > 0) && (r + dr < h) && (dr != 0)) {
                    count++;
                    avg_r += RowAverageRed(result, w, r + dr);
                    avg_g += RowAverageGreen(result, w, r + dr);
                    avg_b += RowAverageBlue(result, w, r + dr);
                }
            }
            avg_r /= count;
            avg_g /= count;
            avg_b /= count;

            if (abs(cur_r - avg_r) > ROW_THRESHOLD) {
                for (int j = 0; j < w; j++) {
                    auto &p = result.Pixels[r * w + j];
                    int red = (unsigned int) p.r;
                    //double rd = (double)p.r;
                    //rd = (rd * avg_r) / cur_r;
                    red = red + (avg_r - cur_r);
                    if (red < 0) red = 0;
                    if (red > 255) red = 255;
                    p.r = (unsigned char) red;
                }
            }
            if (abs(cur_g - avg_g) > ROW_THRESHOLD) {
                for (int j = 0; j < w; j++) {
                    auto &p = result.Pixels[r * w + j];
                    int green = (unsigned int) p.g;
                    //unsigned int green = (unsigned int) p.g;
                    //green = (green * avg_g) / cur_g;
                    green = green + (avg_g - cur_g);
                    if (green < 0) green = 0;
                    if (green > 255) green = 255;
                    p.g = green;
                }
            }
            if (abs(cur_b - avg_b) > ROW_THRESHOLD) {
                for (int j = 0; j < w; j++) {
                    auto &p = result.Pixels[r * w + j];
                    int blue = (unsigned int) p.b;
                    //unsigned int blue = (unsigned int) p.b;
                    //blue = (blue * avg_b) / cur_b;
                    blue = blue + (avg_b - cur_b);
                    if (blue < 0) blue = 0;
                    if (blue > 255) blue = 255;
                    p.b = blue;
                }
            }
        }

    }


    void ProcessShot(Image & result, unsigned char * inputBuffer, int w,
            int h) {
        // the current implementation just replicates the pixel's
        // measurement into the RGB channels of the output image
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                if ((i + j) % 2 == 1) {
                    unsigned char avg = demosiac_extended_cross(inputBuffer, w, h, i, j);
                    unsigned char cur = inputBuffer[i * w + j];

                    if (abs(cur - avg) > PRE_THRESHOLD) {
                        inputBuffer[i * w + j] = avg;
                    }
                }
            }
        }

        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                if ((i + j) % 2 == 0) {
                    unsigned char avg_c = demosiac_extended_cross(inputBuffer, w, h, i, j);
                    unsigned char avg_d = demosiac_diagnal(inputBuffer, w, h, i, j);
                    unsigned char avg = (avg_c / 2) + (avg_d / 2);
                    unsigned char cur = inputBuffer[i * w + j];

                    if (abs(cur - avg) > PRE_THRESHOLD) {
                        inputBuffer[i * w + j] = avg;
                    }
                }
            }
        }
        auto pix = inputBuffer;
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                auto & p = result.Pixels[i * w + j];
                unsigned int r, g, b;
                if ((i + j) % 2 == 0) {
                    // green pixel
                    if (i % 2 == 0) {
                        p.g = pix[i * w + j] * G_COMP;
                        p.b = demosiac_vertical(inputBuffer, w, h, i, j);
                        p.r = demosiac_horizontal(inputBuffer, w, h, i, j);
                    } else {
                        p.g = pix[i * w + j] * G_COMP;
                        p.b = demosiac_horizontal(inputBuffer, w, h, i, j);
                        p.r = demosiac_vertical(inputBuffer, w, h, i, j);
                    }
                } else if (i % 2 == 0) {
                    // red pixel
                    p.r = pix[i * w + j];
                    p.g = demosiac_adj(inputBuffer, w, h, i, j);
                    p.b = demosiac_diagnal(inputBuffer, w, h, i, j);
                } else if (i % 2 == 1) {
                    // blue pixel
                    p.b = pix[i * w + j];
                    p.g = demosiac_adj(inputBuffer, w, h, i, j) * G_COMP;
                    p.r = demosiac_diagnal(inputBuffer, w, h, i, j);
                }

            }
        }

        noise_removal(result, w, h);
        remove_stipes(result, w, h, ROW_BLUR_RANGE);
        remove_stipes(result, w, h, ROW_BLUR_RANGE_LEVEL_2);
        noise_removal(result, w, h);
        noise_removal(result, w, h);

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
