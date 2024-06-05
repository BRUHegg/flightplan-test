#include <iostream>
#include <memory>
#include <string>
#include "fpln/flightplan.hpp"

#define UNUSED(x) (void)(x)


double AC_LAT_DEF = 45.588670483;
double AC_LON_DEF = -122.598150383;


namespace test
{
    class Avionics
    {
    public:
        double ac_lat;
        double ac_lon;
        int legs_sbpg;

        double leg_list_id;
        double seg_list_id;

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        std::shared_ptr<libnav::AwyDB> awy_db;
        std::shared_ptr<libnav::HoldDB> hold_db;

        std::shared_ptr<FlightPlan> fpl;

        std::unordered_map<std::string, std::string> env_vars;

        std::string cifp_dir_path;


        Avionics(std::string apt_dat, std::string custom_apt, std::string custom_rnw,
            std::string fix_data, std::string navaid_data, std::string awy_data,
            std::string hold_data, 
            std::string cifp_path, double def_lat=AC_LAT_DEF, 
            double def_lon=AC_LON_DEF)
        {
            env_vars["ac_lat"] = strutils::double_to_str(def_lat, 8);
            env_vars["ac_lon"] = strutils::double_to_str(def_lon, 8);
            env_vars["legs_sbpg"] = "0";
            leg_list_id = -1;
            seg_list_id = -1;

            cifp_dir_path = cifp_path;

            ac_lat = def_lat;
            ac_lon = def_lon;
            legs_sbpg = 0;

            arpt_db_ptr = 
                std::make_shared<libnav::ArptDB>(apt_dat, custom_apt, custom_rnw);
	        navaid_db_ptr = 
                std::make_shared<libnav::NavaidDB>(fix_data, navaid_data);
            awy_db = std::make_shared<libnav::AwyDB>(awy_data);
            hold_db = std::make_shared<libnav::HoldDB>(hold_data);

            libnav::DbErr err_arpt = arpt_db_ptr->get_err();
            libnav::DbErr err_wpt = navaid_db_ptr->get_wpt_err();
            libnav::DbErr err_nav = navaid_db_ptr->get_navaid_err();
            libnav::DbErr err_awy = awy_db->get_err();
            libnav::DbErr err_hold = hold_db->get_err();

            std::cout << navaid_db_ptr->get_wpt_cycle() << " " <<
                navaid_db_ptr->get_navaid_cycle() << " " << 
                awy_db->get_airac() << " " << hold_db->get_airac() << "\n";

            std::cout << "Fix data base version: " << 
                navaid_db_ptr->get_wpt_version() << "\n";
            std::cout << "Navaid data base version: " << 
                navaid_db_ptr->get_navaid_version() << "\n";

            if(err_arpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airport database\n";
            }
            if(err_wpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load waypoint database\n";
            }
            if(err_nav != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load navaid database\n";
            }
            if(err_awy != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airway database\n";
            }
            if(err_hold != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load hold database\n";
            }

            fpl = std::make_shared<FlightPlan>(arpt_db_ptr, navaid_db_ptr, cifp_dir_path);
        }

        std::vector<list_node_ref_t<fpl_seg_t>> get_seg_list()
        {
            std::vector<list_node_ref_t<fpl_seg_t>> scr_data;
            size_t seg_sz = fpl->get_seg_list_sz();
            seg_list_id = fpl->get_sl_seg(0, seg_sz, &scr_data);

            return scr_data;
        }

        std::vector<list_node_ref_t<leg_list_data_t>> get_legs_list()
        {
            std::vector<list_node_ref_t<leg_list_data_t>> scr_data;
            size_t legs_sz = fpl->get_leg_list_sz();
            leg_list_id = fpl->get_ll_seg(0, legs_sz, &scr_data);

            return scr_data;
        }

        void update()
        {
            update_pos();
        }

        ~Avionics()
        {
            hold_db.reset();
            awy_db.reset();
            navaid_db_ptr.reset();
            navaid_db_ptr.reset();
            arpt_db_ptr.reset();
        }

    private:
        void update_pos()
        {
            bool lat_valid = strutils::is_numeric(env_vars["ac_lat"]);
            bool lon_valid = strutils::is_numeric(env_vars["ac_lon"]);
            bool sbp_vaild = strutils::is_numeric(env_vars["legs_sbpg"]);
            
            if(lon_valid && lat_valid)
            {
                ac_lat = std::stod(env_vars["ac_lat"]);
                ac_lon = std::stod(env_vars["ac_lon"]);
            }
            if(sbp_vaild)
            {
                legs_sbpg = std::stoi(env_vars["legs_sbpg"]);
            }
        }
    };

    struct awy_filter_data_t
    {
        std::string s;
        Avionics* ptr;
    };

    typedef void (*cmd)(Avionics*, std::vector<std::string>&);


    inline void set_var(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: <variable name>, <value>\n";
            return;
        }

        av->env_vars[in[0]] = in[1];
    }

    inline void print(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: <variable name>\n";
            return;
        }

        if(av->env_vars.find(in[0]) != av->env_vars.end())
        {
            std::cout << av->env_vars[in[0]] << "\n";
        }
        else
        {
            std::cout << "Variable not found\n";
        }
    }

    inline void quit(Avionics* av, std::vector<std::string>& in)
    {
        UNUSED(av);

        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        std::exit(0);
    }

    inline void fplinfo(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        std::cout << "Departure: " << av->fpl->get_dep_icao() << "\n";
        std::cout << "Arrival: " << av->fpl->get_arr_icao() << "\n";
    }

    inline void set_fpl_dep(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        libnav::DbErr err = av->fpl->set_dep(in[0]);
        if(err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid entry\n";
        }
        else if(err == libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Airport partially loaded\n";
        }
    }

    inline void set_fpl_arr(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: icao code\n";
            return;
        }

        libnav::DbErr err = av->fpl->set_arr(in[0]);
        if(err != libnav::DbErr::SUCCESS && err != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid entry\n";
        }
        else if(err == libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Airport partially loaded\n";
        }
    }

    inline void add_seg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() < 4)
        {
            std::cout << "Command expects 4 arguments: <seg name>, <seg type>, <next>, <legs>\n";
            return;
        }

        auto seg_mp = av->fpl->get_seg_map();
        fpl_segment_types tp = fpl_segment_types(strutils::stoi_with_strip(in[1]));
        auto seg_list = av->get_seg_list();
        size_t idx = size_t(strutils::stoi_with_strip(in[2]));
        std::vector<int> legs;
        for(size_t i = 3; i < in.size(); i++)
        {
            legs.push_back(strutils::stoi_with_strip(in[i])); //addseg A 2 TAIL 1 2 3
        }
        av->fpl->add_segment(legs, tp, in[0], seg_list[idx].ptr);
    }

    inline void add_legs(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() < 3)
        {
            std::cout << "Command expects 4 arguments: <seg name>, <seg type>, \
            <start leg>, <legs>\n";
            return;
        }
        fpl_segment_types tp = fpl_segment_types(strutils::stoi_with_strip(in[1]));
        std::vector<int> legs;
        int start = strutils::stoi_with_strip(in[2]);
        for(size_t i = 3; i < in.size(); i++)
        {
            legs.push_back(strutils::stoi_with_strip(in[i]));
        }

        av->fpl->add_legs(start, legs, tp, in[0]);
    }

    inline void add_dir(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments: <leg>, <next>\n";
            return;
        }

        int leg = strutils::stoi_with_strip(in[0]);
        size_t idx = size_t(strutils::stoi_with_strip(in[1]));
        auto legs = av->get_legs_list();
        av->fpl->add_direct(leg, legs[idx].ptr);
    }

    inline void del_seg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: <segment index>\n";
            return;
        }
        auto seg_list = av->get_seg_list();
        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        if(idx > 0 && idx < seg_list.size()-1)
            av->fpl->delete_segment(seg_list[idx].ptr);
    }

    inline void del_leg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Command expects 1 argument: <leg index>\n";
            return;
        }

        auto leg_list = av->get_legs_list();
        size_t idx = size_t(strutils::stoi_with_strip(in[0]));
        av->fpl->delete_leg(leg_list[idx].ptr);
    }

    inline void delbe(Avionics* av, std::vector<std::string>& in)
    {
        auto legs_list = av->get_legs_list();
        size_t start = size_t(strutils::stoi_with_strip(in[0]));
        size_t end = size_t(strutils::stoi_with_strip(in[1]));

        av->fpl->delete_range(legs_list[start].ptr, legs_list[end].ptr);
    }

    inline void print_seg(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        auto seg_list = av->get_seg_list();
        size_t cnt = 1;
        for(size_t i = 1; i < seg_list.size()-1; i++)
        {
            list_node_ref_t<fpl_seg_t> tmp = seg_list[i];
            std::cout << cnt << ". Segment " << tmp.data.name << " " << 
                tmp.data.seg_type << "\n";
            std::cout << "End leg: " << tmp.data.end->data.leg << "\n";
            cnt++;
        }
        
        std::cout << "\n";
    }

    // Print commands

    inline void print_legs(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        
        auto legs = av->get_legs_list();
        for(size_t i = 1; i < legs.size()-1; i++)
        {
            list_node_ref_t<leg_list_data_t> tmp = legs[i];
            if(!tmp.data.is_discon)
                std::cout << tmp.data.leg << " ";
            else
                std::cout << DISCON_SEG_NAME << " ";
        }
        std::cout << "\n";
    }

    inline void print_refs(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        av->fpl->print_refs();
    }

    inline void print_cdu_legs(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }

        std::vector<list_node_ref_t<leg_list_data_t>> scr_data;  // What's shown on CDU screen
        av->leg_list_id = av->fpl->get_ll_seg(size_t(av->legs_sbpg) * 5, 5, &scr_data);
        for(size_t i = 0; i < scr_data.size(); i++)
        {
            std::cout << scr_data[i].data.leg << "\n";
        }
    }

    std::unordered_map<std::string, cmd> cmd_map = {
        {"set", set_var},
        {"print", print},
        {"p", print},
        {"quit", quit},
        {"q", quit},
        {"fplinfo", fplinfo},
        {"setdep", set_fpl_dep},
        {"setarr", set_fpl_arr},
        {"addseg", add_seg},
        {"addlegs", add_legs},
        {"addir", add_dir},
        {"delseg", del_seg},
        {"deleg", del_leg},
        {"pseg", print_seg},
        {"plegs", print_legs},
        {"prefs", print_refs},
        {"pcl", print_cdu_legs},
        {"dbe", delbe}
        };
}
