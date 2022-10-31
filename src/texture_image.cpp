
#include <nori/texture.h>
#include <filesystem/resolver.h>
#include <fstream>
#include <nori/stb_image.h>
#include <unordered_map>

NORI_NAMESPACE_BEGIN

class TexImage : public NoriTexture
{
public:
    TexImage() {}

    TexImage(std::unordered_map<std::string, unsigned char *> pixels, int32_t weight, int32_t height)
        : pixels(pixels), weight(weight), height(height) {}

    TexImage(const PropertyList &propList)
    {
        // load  Material library file
        filesystem::path filename =
            getFileResolver()->resolve(propList.getString("mtl"));

        std::ifstream is(filename.str());
        if (is.fail())
            throw NoriException("Unable to open MTL file \"%s\"! (Check if file exists).", filename);
        cout << "Loading \"" << filename << "\" .. ";
        cout.flush();

        std::vector<std::string> image_name_list;
        std::string line_str, mtl_name;
        while (std::getline(is, line_str)) {
            std::istringstream line(line_str);
            std::string prefix;
            line >> prefix;

            if (prefix == "newmtl")
            {
                line >> mtl_name;
                this->mtl_dict[mtl_name].name = mtl_name;
            }
            else if (prefix == "Ns")
            {
                line >> this->mtl_dict[mtl_name].Ns;
            }
            else if (prefix == "Ni")
            {
                line >> this->mtl_dict[mtl_name].Ni;
            }
            else if (prefix == "d")
            {
                line >> this->mtl_dict[mtl_name].d;
            }
            else if (prefix == "Tf")
            {
                line >> this->mtl_dict[mtl_name].Tf.x() >> this->mtl_dict[mtl_name].Tf.y() >> this->mtl_dict[mtl_name].Tf.z();
            }
            else if (prefix == "illum")
            {
                line >> this->mtl_dict[mtl_name].illum;
            }
            else if (prefix == "Ka")
            {
                line >> this->mtl_dict[mtl_name].Ka.x() >> this->mtl_dict[mtl_name].Ka.y() >> this->mtl_dict[mtl_name].Ka.z();
            }
            else if (prefix == "Kd")
            {
                line >> this->mtl_dict[mtl_name].Kd.x() >> this->mtl_dict[mtl_name].Kd.y() >> this->mtl_dict[mtl_name].Kd.z();
            }
            else if (prefix == "Ks")
            {
                line >> this->mtl_dict[mtl_name].Ks.x() >> this->mtl_dict[mtl_name].Ks.y() >> this->mtl_dict[mtl_name].Ks.z();
            }
            else if (prefix == "map_Kd")
            {
                line >> this->mtl_dict[mtl_name].map_Kd;
                image_name_list.push_back(this->mtl_dict[mtl_name].map_Kd);
            }
            else {
                cerr << "\"" << prefix << "\" attribute of Mtl no define!" << endl;
            }
        }
        cout << "done. (mtl_name=" << mtl_name << ")" << endl;


        // load image file
        std::string image_dir = filename.str().substr(0, filename.str().find_last_of('/') + 1);
        for (size_t i = 0; i < image_name_list.size(); ++i)
        {
            filename = getFileResolver()->resolve(image_dir + image_name_list[i]);
            std::ifstream is(filename.str());
            if (is.fail())
                throw NoriException("Unable to open IMAGE file \"%s\"! (Check if file exists).", filename);
            cout << "Loading \"" << filename << "\" .. ";
            cout.flush();
            std::string imgname = image_name_list[i].substr(image_name_list[i].find_last_of('/')+1);
            this->pixels[imgname] = stbi_load(filename.str().c_str(), &weight, &height, &channel, 0);
            cout << "done. (weight=" << weight << ", height=" << height << ", channel=" << channel << " and "
                 << memString(sizeof(uint8_t) * weight * height * channel)
                 << ")" << endl;
        }
    }

    virtual Color3f getValue(std::string mtl_name, float u, float v) const override;
    virtual Color3f  getValue(std::string mtl_name, const Point2f &p) const override;
    std::string toString() const;

private:
    std::unordered_map<std::string, unsigned char *> pixels; // <name, data>
    int32_t weight, height, channel, img_nums;
};

Color3f TexImage::getValue(std::string mtl_name, float u, float v) const
{
    //  operator [] of <STL map container> allows the value to be modified, and
    // it will be added when the key does not exist. Therefore, .at should be used in
    // the const member function to obtain the value corresponding to the current key.
    std::string imgname = this->mtl_dict.at(mtl_name).map_Kd;
    int32_t i = u * weight;
    int32_t j = (1 - v) * height;
    i = i >= 0 ? i : 0;
    j = j >= 0 ? j : 0;
    i = i < weight ? i : weight - 1;
    j = j < height ? j : height - 1;
    float r = int(pixels.at(imgname)[channel * (i + weight * j) + 0]) / 255.f;
    float g = int(pixels.at(imgname)[channel * (i + weight * j) + 1]) / 255.f;
    float b = int(pixels.at(imgname)[channel * (i + weight * j) + 2]) / 255.f;
    return Color3f(r, g, b);
}

Color3f TexImage::getValue(std::string mtl_name, const Point2f &p) const
{
    return getValue(mtl_name, p[0], p[1]);
}

std::string TexImage::toString() const
{
    return tfm::format(
        "weight = \"%d\"\n"
        "height = \"%d\"\n",
        weight, height);
}

NORI_REGISTER_CLASS(TexImage, "texImage");
NORI_NAMESPACE_END