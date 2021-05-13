#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

ros::Publisher output_pub;

struct Exclude
{
    Exclude(double min, double max)
    {
        min_radius = min;
        max_radius = max;
        min_radius_squared = min*min;
        max_radius_squared = max*max;
    }

    double min_radius;
    double max_radius;
    double min_radius_squared;
    double max_radius_squared;

    bool operator()(double radius_squared) const
    {
        return (radius_squared >= min_radius_squared && radius_squared <= max_radius_squared);
    }
};

std::vector<Exclude> excludes;

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
            float x = *reinterpret_cast<const float*>(&data->data[i+xoffset]);
            float y = *reinterpret_cast<const float*>(&data->data[i+yoffset]);
            float z = *reinterpret_cast<const float*>(&data->data[i+zoffset]);
            double r2 = x*x+y*y+z*z;
            bool exclude = false;
            for(auto e: excludes)
            {
                exclude = exclude || e(r2);
                if(exclude)
                    break;
            }
            if (!exclude)
            {
                ROS_DEBUG_STREAM("keeping: " << x << ", " << y << ", " << z << " r: " << sqrt(r2));
                for(uint32_t j = i; j < i+data->point_step; j++)
                    out.data.push_back(data->data[j]);
                out.row_step += data->point_step;
                out.width += 1;
            }
        }
        output_pub.publish(out);
    }
    else
        ROS_WARN_STREAM("Can't filter: Did not fine xyz fields of type float32 or data is big endian");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud2_spherical_filter");

    if(ros::param::has("~radius"))
    {
        double radius;
        ros::param::param<double>("radius", radius, 1.0);
        excludes.push_back(Exclude(0,radius));
    }

    if(ros::param::has("~excludes"))
    {
        XmlRpc::XmlRpcValue excludes_list;
        ros::param::get("~excludes", excludes_list);
        if(excludes_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int32_t i = 0; i < excludes_list.size(); ++i) 
            {
                double min,max;
                min = static_cast<double>(excludes_list[i]["min"]);
                max = static_cast<double>(excludes_list[i]["max"]);
                excludes.push_back(Exclude(min,max));
                ROS_INFO_STREAM("Adding exclude: " << min << " - " << max);
            }
        }
        else
            ROS_WARN_STREAM("Expected excludes paramter to be a list, got " << excludes_list.getType());
    }   

    ros::NodeHandle nh;

    output_pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);
    ros::Subscriber input_sub = nh.subscribe("input", 1, &pc_callabck);
    
    ros::spin();
    
    return 0;
}

