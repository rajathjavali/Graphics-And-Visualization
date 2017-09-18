/** Class implementing the bar chart view. */
class BarChart {

    /**
     * Create a bar chart instance and pass the other views in.
     * @param worldMap
     * @param infoPanel
     * @param allData
     */
    constructor(worldMap, infoPanel, allData) {
        this.worldMap = worldMap;
        this.infoPanel = infoPanel;
        this.allData = allData;
    }

	findMax(d)
	{
		let max = 0;
		for(let iter of this.allData)
		{
			if(iter[d] > max)
				max = iter[d];
		}
		return max;
	}
    /**
     * Render and update the bar chart based on the selection of the data type in the drop-down box
     */
    updateBarChart(selectedDimension) {


        // ******* TODO: PART I *******

		console.log(this.allData[1][selectedDimension]);
        // Create the x and y scales; make
		//let aScale = d3.scaleLinear()
		//	.domain([0, d3.max(data, d => d.a)])
		//	.range([0, 150]);
		
		let maxOfData = this.findMax(selectedDimension);
		let xScale = d3.scaleLinear()
			.domain([0, (this.allData).length])
			.range([20, 400]);
			
		let yScale = d3.scaleLinear()
			.domain([0, d3.max(maxOfData, d => d)])
			.range([20, 400]);
			
        // sure to leave room for the axes

        // Create colorScale

        // Create the axes (hint: use #xAxis and #yAxis)

        // Create the bars (hint: use #bars)
		let barGraph = d3.select("#bars");
		let bars = barGraph.select("rect").data(this.allData);
		
		bars.enter().append("rect")
			.attr("x", (d, i) => xScale(i))
			.attr("y", 0)
			.attr("width", 10)
			.attr("height", d => yScale(d[selectedDimension]))
			.style("fill", "steelblue")
			.attr("opacity", 1);
		//.transition()
		//.duration(3000)
		//.attr("height", d => yScale(d[selectedDimension]))
		//.d => colorScale(getData(d, selectedDimension)));
		
		
        // ******* TODO: PART II *******

        // Implement how the bars respond to click events
        // Color the selected bar to indicate is has been selected.
        // Make sure only the selected bar has this new color.

        // Call the necessary update functions for when a user clicks on a bar.
        // Note: think about what you want to update when a different bar is selected.

    }

    /**
     *  Check the drop-down box for the currently selected data type and update the bar chart accordingly.
     *
     *  There are 4 attributes that can be selected:
     *  goals, matches, attendance and teams.
     */
    chooseData() {
        // ******* TODO: PART I *******
        //Changed the selected data when a user selects a different
        // menu item from the drop down.

    }
}