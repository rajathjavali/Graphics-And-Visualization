/** Class implementing the map view. */
class Map {
    /**
     * Creates a Map Object
     */
    constructor() {
        this.projection = d3.geoConicConformal().scale(150).translate([400, 350]);

    }

    /**
     * Function that clears the map
     */
    clearMap() {

        // ******* TODO: PART V*******
        // Clear the map of any colors/markers; You can do this with inline styling or by
        // defining a class style in styles.css

        // Hint: If you followed our suggestion of using classes to style
        // the colors and markers for hosts/teams/winners, you can use
        // d3 selection and .classed to set these classes on and off here.
		d3.select("#map").selectAll(".host").classed("host", false).classed("countries", true);
		d3.select("#map").selectAll(".team").classed("team", false).classed("countries", true);
		
		d3.select("#points").selectAll("circle").remove();
    }

    /**
     * Update Map with info for a specific FIFA World Cup
     * @param wordcupData the data for one specific world cup
     */
    updateMap(worldcupData) {

        //Clear any previous selections;
        this.clearMap();

        // ******* TODO: PART V *******

        // Add a marker for the winner and runner up to the map.

        // Hint: remember we have a conveniently labeled class called .winner
        // as well as a .silver. These have styling attributes for the two
        // markers.

		// We strongly suggest using CSS classes to style the selected countries.
		let map = d3.select("#map");
		
		//let team  = map.data(worldcupData.teams_iso).selectAll("path")
		//team.attr("id", function(d){return "#"+d;}).attr("class", "team");
		// Iterate through all participating teams and change their color as well.
		
		//let acdsfds = map.data(worldcupData.teams_iso).select(d => "#" + d).attr("class", "team");
		let team, i = 0;
		for(let iter of worldcupData.teams_iso)
		{
			team = "#"+iter;
			map.select(team).attr("class", "team")
		}
		
		// Select the host country and change it's color accordingly.
		let selection = "#" + worldcupData.host_country_code;
		map.select(selection).attr("class", "host");
		let globeProj = this.projection;
        // Add a marker for gold/silver medalists
		let markers = d3.select("#points");
		
		//winner
		markers.append("circle")
				.attr("cx", function (d) {
					return globeProj([worldcupData.win_pos[0], worldcupData.win_pos[1]])[0];
				})
				.attr("cy", function (d) {
					return globeProj([worldcupData.win_pos[0], worldcupData.win_pos[1]])[1];
				})
				.attr("r", 8)
				.classed("gold", true);
				
		//runner_up
		markers.append("circle")
				.attr("cx", function (d) {
					return globeProj([worldcupData.ru_pos[0], worldcupData.ru_pos[1]])[0];
				})
				.attr("cy", function (d) {
					return globeProj([worldcupData.ru_pos[0], worldcupData.ru_pos[1]])[1];
				})
				.attr("r", 8)
				.classed("silver", true);
    }

    /**
     * Renders the actual map
     * @param the json data with the shape of all countries
     */
    drawMap(world) {

        //(note that projection is a class member
        // updateMap() will need it to add the winner/runner_up markers.)

        // ******* TODO: PART IV *******

        // Draw the background (country outlines; hint: use #map)
        // Make sure and add gridlines to the map

        // Hint: assign an id to each country path to make it easier to select afterwards
        // we suggest you use the variable in the data element's .id field to set the id

        // Make sure and give your paths the appropriate class (see the .css selectors at
        // the top of the provided html file)

        
        // Load in GeoJSON data
	
		// Bind data and create one path per GeoJSON feature
		
		let barChart =  window.barChart;
		let infoPane = window.infoPanel;
		
		
		
		let path = d3.geoPath()
            .projection(this.projection);
		
		let map = d3.select("#map").selectAll("path")
			.data(topojson.feature(world, world.objects.countries).features)
			.enter()
			.append("path")
			.attr("d", path)
			.attr("id", d => d.id)
			.classed("countries", true)
			.on("click", function(d) { 
				let participated = [], host = [], k =0; //0 - hosted, 1-participated
				participated[0] = "Never Participated";
				for(let i of barChart.allData)
				{					
					for(let j of i.teams_iso)
					{
						if(d.id == j)
						{
							participated[k] = i.year;
							host[k] = 0;
							console.log(i.year);
							if(d.id == i.host_country_code)
								host[k] = 1;
							k++;
							break;
						}
					}
				}
				infoPane.updateMapData(d.id, participated, host);
			});

		let graticule = d3.geoGraticule();
		d3.select("#map").append('path').datum(graticule).attr('class', "grat").attr('d', path).attr('fill', 'none');
    
    }

}