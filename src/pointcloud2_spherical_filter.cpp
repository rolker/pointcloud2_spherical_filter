#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

ros::Publisher output_pub;
double radius;
double radius_squared;

void pc_callabck(const sensor_msgs::PointCloud2::ConstPtr &data)
{
    int xoffset = -1;
    int yoffset = -1;
    int zoffset = -1;
    for(auto field: data->fields)
    {
        if (field.datatype == 7) // only support float32 for now
        {
            if(field.name == "x")
                xoffset = field.offset;
            if(field.name == "y")
                yoffset = field.offset;
            if(field.name == "z")
                zoffset = field.offset;
        }
    }
    if(xoffset >= 0 && yoffset >= 0 && zoffset >= 0 && !data->is_bigendian)
    {
        sensor_msgs::PointCloud2 out;
        out.header = data->header;
        out.height = data->height;
        out.width = 0;
        for(auto field: data->fields)
            out.fields.push_back(field);
        out.is_bigendian = data->is_bigendian;
        out.point_step = data->point_step;
        out.row_step = 0;
        out.is_dense = data->is_dense;
        
        for(uint32_t i = 0; i+data->point_step <= data->data.size(); i+=data->point_step)
        {
            double x = *reinterpret_cast<const double*>(&data->data[i+xoffset]);
            double y = *reinterpret_cast<const double*>(&data->data[i+yoffset]);
            double z = *reinterpret_cast<const double*>(&data->data[i+zoffset]);
            double r2 = x*x+y*y+z*z;
            if (r2 > radius_squared)
            {
                for(uint32_t j = i; j < i+data->point_step; j++)
                    out.data.push_back(data->data[j]);
                out.row_step += data->point_step;
                out.width += 1;
            }
        }
        output_pub.publish(out);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud2_spherical_filter");
    ros::NodeHandle nh;

    nh.param<double>("radius", radius, 1.0);
    radius_squared = radius*radius;
    
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("/output",1);
    ros::Subscriber input_sub = nh.subscribe("/input", 1, &pc_callabck);
    
    ros::spin();
    
    return 0;
}

