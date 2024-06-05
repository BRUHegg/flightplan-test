#include "main_helpers.hpp"


int main()
{
    std::string earth_nav_path = "/home/betatest/X-Plane 12/Resources/default data/";
    std::string apt_path = "/home/betatest/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
    
    test::Avionics avncs(apt_path, "777_arpt.dat", 
		"777_rnw.dat", earth_nav_path+"earth_fix.dat", 
		earth_nav_path+"earth_nav.dat", earth_nav_path+"earth_awy.dat", 
		earth_nav_path+"earth_hold.dat", earth_nav_path+"CIFP");
	std::cout << "Avionics loaded\n";

	std::vector<std::string> pre_exec = {
		"addlegs RW10 1 1",
		"addlegs NOB 2 2 3 4",
		"addlegs AWY 4 4 5 6 7 8",
		"addlegs KOK 2 9 10 11",
		"addlegs NOB 2 2 3 4",
		"addlegs AWY2 4 8 9 10 11",
		"addir 12 9",
		"addir 13 10",
		"dbe 10 12",
		"addir 12 5",
		"addlegs KOK 2 15 16 17",
		"addlegs KOM 2 18 19 12",
		"plegs",
		"pseg",
		"prefs"
		};
	//pre_exec = {};

	size_t i = 0;

    while(1)
	{
		std::string in_raw;
		if(i < pre_exec.size())
		{
			in_raw = pre_exec[i];
			i++;
		}
		else
		{
			std::cout << ">> ";
			std::getline(std::cin, in_raw);
		}
		
		
		std::string in_proc = strutils::strip(in_raw, ' ');

		std::vector<std::string> line_split = strutils::str_split(in_proc, ' ');
		std::string cmd_name = line_split[0];
		std::vector<std::string> args = std::vector<std::string>(line_split.begin() + 1, 
			line_split.end());

		if(test::cmd_map.find(cmd_name) != test::cmd_map.end())
		{
			test::cmd_map[cmd_name](&avncs, args);
		}
		else
		{
			std::cout << "Invalid command name\n";
		}


		avncs.update();
	}
}
