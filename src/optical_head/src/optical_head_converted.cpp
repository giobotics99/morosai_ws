#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <optical_head/msg/pgv_scan_data.hpp>
#include <optical_head/msg/pgv_dir_msg.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <bitset>
#include <string>
#include <functional>
#include <stdlib.h>
#include <signal.h>
#include <sstream>

// Funzione per gestire l'interruzione da CTRL + C
void my_handler(int s);
int serial_port;
unsigned char dir_straight[2] = {0xEC, 0x13};
unsigned char dir_left[2] = {0xE8, 0x17};
unsigned char dir_right[2] = {0xE4, 0x1B};
unsigned char dir_nolane[2] = {0xE0, 0x1F};
unsigned char pos_req[2] = {0xC8, 0x37};

// subito dopo gli #include, prima di class AGVNode
unsigned long string2decimal(const std::string &input) {
    return strtoull(input.c_str(), nullptr, 2);
}

optical_head::msg::PgvDirMsg sub_direction;

class AGVNode : public rclcpp::Node
{

private:

public:

    std::string selected_dir;
    void direction_callback(const optical_head::msg::PgvDirMsg::SharedPtr msg) {
    sub_direction.dir_command = msg->dir_command;
    switch (sub_direction.dir_command) {
        case 0: selected_dir = "No lane is selected"; write(serial_port, dir_nolane, 2); break;
        case 1: selected_dir = "Right lane is selected"; write(serial_port, dir_right, 2); break;
        case 2: selected_dir = "Left lane is selected"; write(serial_port, dir_left, 2); break;
        case 3: selected_dir = "Straigh Ahead"; write(serial_port, dir_straight, 2); break;
    }
    // RCLCPP_INFO(this->get_logger(), "direction_callback: dir_command=%d selected_dir='%s'", sub_direction.dir_command, selected_dir.c_str());
}

    std::string get_warning_msg(int idx) {
    const char* messages[] = {
        "Code with content not typical of PGV found ",
        "Read head too close to code tape ",
        "Read head too far from code tape ",
        "Reserved", "Reserved",
        "The read head is rotated or tipped in relation to the code tape ",
        "Low level of code contrast ",
        "Repair tape detected ",
        "Temperature too high ",
        "Position code near branch/crossover detected ",
        "More than the specified number of code lanes present ",
        "Reserved", "Reserved", "Reserved"
    };
    return (idx < 14) ? messages[idx] : "NO WARNING";
}

    std::string frame_id_;

    AGVNode() : Node("pgv100_node")
    {
        // Parameter configuration
        this->declare_parameter<std::string>("frame_id", "base_footprint");
        this->get_parameter("frame_id", frame_id_);

        // Configurazione seriale
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        std::string port;
        this->get_parameter("serial_port", port);
        serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s: %s", port.c_str(), strerror(errno));
            throw std::runtime_error("Serial port open failed");
        }
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        }

        // Configurazione seriale
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CS7;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 2;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
        }

    // Impostiamo i publisher ROS
    chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // Publisher that follows the optical_head message schema so other nodes can subscribe
    pgv_pub_ = this->create_publisher<optical_head::msg::PgvScanData>("pgv100_scan", 10);
    // inside AGVNode() constructor, dopo i publisher
    dir_sub_ = this->create_subscription<optical_head::msg::PgvDirMsg>(
        "pgv_dir", 10, std::bind(&AGVNode::direction_callback, this, std::placeholders::_1));

        // Gestiamo il CTRL + C
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = my_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
    }

    void spin()
    {
        // Dati da inviare e ricevere dal seriale
        unsigned char dir_straight[2] = {0xEC, 0x13}; // Diritto
        unsigned char pos_req[2] = {0xC8, 0x37}; // Richiesta posizione

        write(serial_port, dir_straight, sizeof(dir_straight));
        RCLCPP_INFO(this->get_logger(), "Direction set to <> Straight Ahead <>");

        rclcpp::WallRate loop_rate(10);

        while (rclcpp::ok()) {
            write(serial_port, pos_req, 2);
            char read_buf[256];
            memset(&read_buf, '\0', sizeof(read_buf));
            int byte_count = read(serial_port, &read_buf, sizeof(read_buf));

            // Otteniamo l'angolo dal byte array [Byte 11-12]
            std::bitset<7> ang_1(read_buf[10]);
            std::bitset<7> ang_0(read_buf[11]);
            std::string agv_ang_str = ang_1.to_string() + ang_0.to_string();
            int strlength = agv_ang_str.length();
            char agv_ang_char[strlength + 1];
            strcpy(agv_ang_char, agv_ang_str.c_str());
            char *angEnd;
            float agv_ang_des = strtoull(agv_ang_char, &angEnd, 2);
            double agv_x_pos_des = 0.0, agv_y_pos_des;
            int tag_detected_des = 0;

            // Publish string (legacy) for backward compatibility
            std_msgs::msg::String msg;
            std::stringstream ss;
            ss << agv_ang_des / 10;
            msg.data = ss.str();
            chatter_pub_->publish(msg);


            // Get Lane-Detection from the byte array [Bytes 1-2]
            std::bitset<7> lane_detect_byte1(read_buf[0]);
            std::bitset<7> lane_detect_byte0(read_buf[1]);
            std::string agv_lane_detect_str = lane_detect_byte1.to_string() + lane_detect_byte0.to_string();
            std::string agv_c_lane_count_str = agv_lane_detect_str.substr(8, 2);
            std::string agv_c_lane_detect_str = agv_lane_detect_str.substr(11, 1);
            std::string agv_no_pos_str = agv_lane_detect_str.substr(5, 1);
            std::string tag_detected = agv_lane_detect_str.substr(7,1);
            int agv_c_lane_count_des = string2decimal(agv_c_lane_count_str);
            int agv_no_color_lane_des = string2decimal(agv_c_lane_detect_str);
            int agv_no_pos_des = string2decimal(agv_no_pos_str);
            tag_detected_des = string2decimal(tag_detected); 

            // Get the X-Position from the byte array [Bytes 3-4-5-6]
            std::bitset<3> x_pos_3(read_buf[2]);
            std::bitset<7> x_pos_2(read_buf[3]);
            std::bitset<7> x_pos_1(read_buf[4]);
            std::bitset<7> x_pos_0(read_buf[5]);
            std::string agv_x_pos_str;
            // X può essere con o senza segno
            // si valutano i due casi
            if(tag_detected_des != 0){
                //Signed
                if(x_pos_3[2])
                    agv_x_pos_str = "11111111" + x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();
                else
                    agv_x_pos_str = "00000000" + x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();
            }
            else
                agv_x_pos_str = "00000000" + x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();

            std::bitset<32> bs(agv_x_pos_str);
            unsigned long num = bs.to_ulong();
            agv_x_pos_des = (int)num;

            
            // Get Y-Position from the byte array [Bytes 7-8]
            std::bitset<7> y_pos_1(read_buf[6]);
            std::bitset<7> y_pos_0(read_buf[7]);
            std::string agv_y_pos_str;

            // Y è con segno, pertanto bisogna rendere il bitset di 16 bit replicando il MSB
            if(y_pos_1[6])
                agv_y_pos_str = "11" + y_pos_1.to_string() + y_pos_0.to_string();
            else
                agv_y_pos_str = "00" + y_pos_1.to_string() + y_pos_0.to_string();

            std::bitset<16> bs_short(agv_y_pos_str);
            num = bs_short.to_ulong();
            agv_y_pos_des = (short)num;

            
            // We get opposite values when we try the read the y-pos value from the colored and code strip.
            // So this is checking which strip that we're reading.
            if(agv_no_pos_des)
            agv_y_pos_des *= -1;
            
            // Get Tag ID from the byte array [Bytes 13-16]
            // unsigned long tag_id = 0;  // dichiarata prima del ciclo if

            // if(byte_count >= 16) { // almeno 16 byte letti
            //     std::bitset<7> b3(read_buf[12] & 0x7F); // MSB ignorato
            //     std::bitset<7> b2(read_buf[13] & 0x7F);
            //     std::bitset<7> b1(read_buf[14] & 0x7F);
            //     std::bitset<7> b0(read_buf[15] & 0x7F); // LSB

            //     std::string tag_bits = b3.to_string() + b2.to_string() + b1.to_string() + b0.to_string();

            //     unsigned long tag_id = std::bitset<28>(tag_bits).to_ulong();

            //     if(tag_id != 0) {
            //         RCLCPP_INFO(this->get_logger(), "Detected Tag ID: %lu", tag_id);
            //     } else {
            //         RCLCPP_INFO(this->get_logger(), "Tag bytes present but ID is 0");
            //     }
            // }

            // RCLCPP_INFO(this->get_logger(), "Tag bytes: %02X %02X %02X %02X", 
            // read_buf[12], read_buf[13], read_buf[14], read_buf[15]);


            // if(tag_detected_des != 0 && byte_count >= 17) {
            //     uint32_t detected_tag_id = 0;
            //     for(int i=0; i<4; i++) {
            //         detected_tag_id <<= 7;          // shift dei 7 bit precedenti
            //         detected_tag_id |= (read_buf[12 + i] & 0x7F);  // aggiungi i 7 bit del byte corrente
            //     }
            //     RCLCPP_INFO(this->get_logger(), "Detected Tag ID: %u", detected_tag_id);
            // } else {
            //     RCLCPP_INFO(this->get_logger(), "No tag detected in this scan");
            // }


            bool err = lane_detect_byte1[0];
            bool war = lane_detect_byte1[2];

                
            int N_warnings = 0;
            std::string warn_string = "";

            std::bitset<7> warning_1(read_buf[19]); // 20esimo byte
            for(int i=0; i<7; i++)
                if(warning_1[i])
                    N_warnings++, warn_string += get_warning_msg(i);
                    

            std::bitset<7> warning_2(read_buf[18]); // 19esimo byte
            for(int i=0; i<7; i++)
                if(warning_2[i])
                    N_warnings++, warn_string += get_warning_msg(i+7);

            if(err)
                warn_string = "ERROR BIT 1 - " + warn_string + " - NumWarnings " + std::to_string(N_warnings);
            else if(war)
                warn_string = "WARNING BIT 1 - " + warn_string + " - NumWarnings " + std::to_string(N_warnings);
            else
                warn_string = warn_string + " - NumWarnings " + std::to_string(N_warnings);

            // Also publish a typed PgvScanData message so /pgv100_scan is available
            optical_head::msg::PgvScanData pmsg;

            pmsg.header.stamp = this->now();
            pmsg.header.frame_id = frame_id_;
            pmsg.angle = agv_ang_des; 
            pmsg.x_pos = agv_x_pos_des;
            pmsg.y_pos = agv_y_pos_des;
            pmsg.direction = selected_dir;
            pmsg.color_lane_count = agv_c_lane_count_des;
            pmsg.no_color_lane = agv_no_color_lane_des;
            pmsg.no_pos = agv_no_pos_des;
            // pmsg.tag_detected = tag_detected_des;
            pmsg.warning_string = warn_string;
            pmsg.error = err;
            // pmsg.tag_id = 0;

            pgv_pub_->publish(pmsg);

            // bool publish_flag = !err; // && !war && !N_warnings; // Check sui bit di errore/warning
            // publish_flag &= (abs(pmsg.angle) <= 180.0); // Check angolo
            // publish_flag &= (byte_count == 21); // Succede che legga meno di 21 byte (3)
            // // publish_flag &= !agv_no_pos_des; // Controlla che la posizione in X sia disponibile
            // publish_flag &= agv_no_color_lane_des; // Controlla che non sia identificata la striscia a colori

            // if(publish_flag)
            //     pgv_pub_->publish(pmsg);

            rclcpp::spin_some(shared_from_this());
            loop_rate.sleep();
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
    rclcpp::Publisher<optical_head::msg::PgvScanData>::SharedPtr pgv_pub_;
    rclcpp::Subscription<optical_head::msg::PgvDirMsg>::SharedPtr dir_sub_;
};

void my_handler(int s)
{
    close(serial_port);
    printf("\n\nCaught signal %d\n Port closed.\n", s);
    exit(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AGVNode>();
    node->spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//  ls -l /dev/serial/by-id 
// ls -l /dev/pgv100
