#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <cctype>
#include <algorithm>

// 三次元座標を表す構造体
struct Point3D {
    double x, y, z;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// gpsのデータを扱うクラス
class gps_point {
private:
    double latitude;        // 度単位
    double longitude;       // 度単位
    double altitude;        // メートル

public:
    gps_point(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}
    
    // 度を取得するゲッター
    double getLatitude() const { return latitude; }
    double getLongitude() const { return longitude; }
    double getAltitude() const { return altitude; }

    // 度を直交座標に変換する関数
    Point3D toCartesian() const {
        const double EARTH_RADIUS = 6378137;    // メートル

        double radLat = latitude * M_PI / 180.0;
        double radLon = longitude * M_PI / 180.0;

        double x = (EARTH_RADIUS + altitude) * cos(radLat) * cos(radLon);
        double y = (EARTH_RADIUS + altitude) * cos(radLat) * sin(radLon);
        double z = (EARTH_RADIUS + altitude) * sin(radLat);

        return Point3D(x, y, z);
    }
};

// CSVファイルを読み込み、gps_pointオブジェクトのベクターを作成する関数
std::vector<gps_point> readCSV(const std::string& path) {
    std::vector<gps_point> data;
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "ファイルを開けませんでした: " << path << std::endl;
    }
    
    std::string line;
    // ヘッダーのスキップ
    if (std::getline(file, line)) {
        std::cout << "スキップしたヘッダー: " << line << std::endl;
    }

    // データの読み込み
    while (std::getline(file, line)) {
        std::istringstream ss(line);        // 文字列ストリームを使ってカンマ区切りを処理
        std::string item;
        double lat = 0.0, lon = 0.0, alt = 0.0;

        if (std::getline(ss, item, ',')) lon = std::stof(item);
        if (std::getline(ss, item, ',')) lat = std::stof(item);
        if (std::getline(ss, item, ',')) alt = std::stof(item);

        std::getline(file, line);

        data.emplace_back(lat, lon, alt);
    }
    file.close();
    return data;
}

// メイン文
int main() {
    std::string path = "C:\\drone_cpp\\GPS_data\\LOG00004.csv";
    std::vector<gps_point> data = readCSV(path);

    if (data.empty()) {
        std::cerr << "ファイルを開けませんでした: " << path << std::endl;
        return 1;
    }

    // データの出力
    for (const auto& gps : data) {
        std::cout << "Longitude: " << gps.getLatitude()
                  << ", Latitude: " << gps.getLongitude()
                  << ", Altitude: " << gps.getAltitude() << std::endl;
        
        Point3D cartesian = gps.toCartesian();
        std::cout << "Cartesian Coordinates: ("
                  << cartesian.x << ", " << cartesian.y << ", " << cartesian.z << ")\n";
    }

    return 0;
}