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
	
	findMin(max, d)
	{
		let min = max;
		for(let iter of this.allData)
		{

			if(iter[d] < min)
				min = iter[d];
		}
		return min;
	}
    /**
     * Render and update the bar chart based on the selection of the data type in the drop-down box
     */
	 
    updateBarChart(selectedDimension) {


        // ******* TODO: PART I *******
		let worldMap = this.worldMap;
		let infPane = this.infoPanel;
		let svg = d3.select("#barChart");
		let length = this.allData.length;
		let height = svg.attr("height");
		let width = svg.attr("width");
		let padding = 50;
		
		let maxOfData = this.findMax(selectedDimension);
		let minOfData = this.findMin(maxOfData, selectedDimension);
		let years = [], selectionData = [], i = 0;
		for(let iter of this.allData)
		{
			years[i++] = iter.year;
		}
		years.sort();
		i = 0;
		for(let iter of this.allData)
		{
			selectionData[i++] = iter[selectedDimension];
		}
		selectionData.sort(d3.descending);
		
		let xScale = d3.scaleLinear()
			.domain([0, length-1])
			.range([width-70, 5]);
			
		let yScale = d3.scaleLinear()
			.domain([0, maxOfData])
			.range([0, height - padding]);
		
		let xAxisScale = d3.scaleBand()
			.domain(years)
			.range([3, width - padding]);
		
		let yAxisScale = d3.scaleLinear()
			.domain([maxOfData, 0])
			.range([0, height - padding]);
		
        // sure to leave room for the axes
        // Create colorScale
		let colorScale = d3.scaleLinear()
            .domain([minOfData, maxOfData])
            .range(["steelblue", "DarkBlue"]);//"#000080"]);//navy
		
		
        // Create the axes (hint: use #xAxis and #yAxis)
		let xAxisSvg = d3.select("#xAxis");
		let xAxis = d3.axisBottom().scale(xAxisScale);
		xAxisSvg.attr("transform", "translate("+ padding +","+ (height - padding) + ")").call(xAxis)
		.selectAll("text")	
        .style("text-anchor", "end")
        .attr("dx", "-.8em")
        .attr("dy", "-.2em")
        .attr("transform", "rotate(-90)");;
		 
		let yAxisSvg = d3.select("#yAxis");
		let yAxis = d3.axisLeft().scale(yAxisScale);
		yAxisSvg.attr("transform", "translate( "+ padding +", 0)")
		.transition()
		.duration(2000)
		.call(yAxis)
		.selectAll("text")
		.style("text-anchor", "end")
        .attr("dx", "0.2em")
        .attr("dy", "0.25em");
		
		let deselect = function(){
			d3.select("#bars").selectAll("rect").style("fill", function (d) {
                return colorScale(d[selectedDimension]);
            });
		}
		
        // Create the bars (hint: use #bars)
		let barGraph = d3.select("#bars");
		let bars = barGraph.selectAll("rect").data(this.allData);
		
		let newBars = bars.enter().append("rect");
		
		
		
		newBars
			.on("click", function(d){
				deselect();
				d3.select(this).style("fill", "red");
				infPane.updateInfo(d);
				worldMap.updateMap(d);
			})
			.attr("x", (d, i) => xScale(i))
			.attr("y", 0)
			.attr("width", 20)
			.attr("height", 0)
			.attr("transform", "translate("+ padding +","+ (height - padding) + ") scale(1, -1)")
			.style("fill", function (d) {
                return colorScale(d[selectedDimension]);
            })
			.attr("opacity", 1);
		
		
		bars = bars.merge(newBars);

		bars
			.transition()
			.duration(2000)
			.attr("height", d => yScale(d[selectedDimension]));
		
		
		
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
		let dataValue = document.getElementById('dataset').value;
		this.updateBarChart(dataValue);
    }
}
