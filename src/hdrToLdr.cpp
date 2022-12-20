#include <nori/bitmap.h>
#include <filesystem/path.h>

int main(int argc, char **argv) {
    using namespace nori;

    try {

        // if file is passed as argument, handle it
        if (argc == 2) {
            std::string filename = argv[1];
            filesystem::path path(filename);

            if (path.extension() == "exr") {
                Bitmap bitmap(filename);

                size_t lastdot = filename.find_last_of(".");
                std::string pngFilename = filename.substr(0, lastdot) + ".png";
                bitmap.saveToLDR(pngFilename);
            } else {
                cerr << "Error: unknown file \"" << filename
                << "\", expected an extension of type .exr" << endl;
            }
        }


    } catch (const std::exception &e) {
        cerr << "Fatal error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
