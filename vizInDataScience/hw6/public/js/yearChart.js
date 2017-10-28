
class YearChart {

    /**
     * Constructor for the Year Chart
     *
     * @param electoralVoteChart instance of ElectoralVoteChart
     * @param tileChart instance of TileChart
     * @param votePercentageChart instance of Vote Percentage Chart
     * @param electionInfo instance of ElectionInfo
     * @param electionWinners data corresponding to the winning parties over mutiple election years
     */
    constructor (electoralVoteChart, tileChart, votePercentageChart, electionWinners, shiftChart) {

        //Creating YearChart instance
        this.electoralVoteChart = electoralVoteChart;
        this.tileChart = tileChart;
        this.votePercentageChart = votePercentageChart;
        this.shiftChart = shiftChart;
        // the data
        this.electionWinners = electionWinners;
        
        // Initializes the svg elements required for this chart
        this.margin = {top: 10, right: 20, bottom: 30, left: 50};
        let divyearChart = d3.select("#year-chart").classed("fullView", true);

        //fetch the svg bounds
        this.svgBounds = divyearChart.node().getBoundingClientRect();
        this.svgWidth = this.svgBounds.width - this.margin.left - this.margin.right;
        this.svgHeight = 100;

        //add the svg to the div
        this.svg = divyearChart.append("svg")
            .attr("width", this.svgWidth)
            .attr("height", this.svgHeight)
    };


    /**
     * Returns the class that needs to be assigned to an element.
     *
     * @param party an ID for the party that is being referred to.
     */
    chooseClass (data) {
        if (data == "R") {
            return "yearChart republican";
        }
        else if (data == "D") {
            return "yearChart democrat";
        }
        else if (data == "I") {
            return "yearChart independent";
        }
    }

    /**
     * Creates a chart with circles representing each election year, populates text content and other required elements for the Year Chart
     */
    update () {

        let _this = this;
        let minyr = d3.min(_this.electionWinners, function(d){ return d.YEAR;});
        let maxyr = d3.max(_this.electionWinners, function(d){ return d.YEAR;});

        let yrScale = d3.scaleLinear()
                        .domain([minyr, maxyr])
                        .range([_this.margin.left, _this.svgWidth - _this.margin.right]);

        //Domain definition for global color scale
        let domain = [-60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60];

        //Color range for global color scale
        let range = ["#063e78", "#08519c", "#3182bd", "#6baed6", "#9ecae1", "#c6dbef", "#fcbba1", "#fc9272", "#fb6a4a", "#de2d26", "#a50f15", "#860308"];

        //Global colorScale be used consistently by all the charts
        this.colorScale = d3.scaleQuantile()
            .domain(domain)
            .range(range);

        // ******* TODO: PART I *******

        //Style the chart by adding a dashed line that connects all these years.
        //HINT: Use .lineChart to style this dashed line
        _this.svg.append("line").attr("x1", 0)
                .attr("y1", _this.svgHeight / 2)
                .attr("x2", _this.svgWidth)
                .attr("y2", _this.svgHeight / 2)
                .attr("class", "lineChart");

        // Create the chart by adding circle elements representing each election year
        //The circles should be colored based on the winning party for that year
        //HINT: Use the .yearChart class to style your circle elements
        //HINT: Use the chooseClass method to choose the color corresponding to the winning party.
        
        //Clicking on any specific year should highlight that circle and  update the rest of the visualizations
        //HINT: Use .highlighted class to style the highlighted circle

        //Election information corresponding to that year should be loaded and passed to
        // the update methods of other visualizations

        let yearData = _this.svg.selectAll("circle").data(_this.electionWinners);
        let circles  = yearData.enter().append("circle").attr("id", d=>d.YEAR);

        yearData.exit().remove();
        yearData = circles.merge(yearData);

        yearData.attr("cx", d=> yrScale(d.YEAR))
                .attr("cy", _this.svgHeight / 2)
                .attr("r", _this.svgHeight / 10)
                .attr("class", d=>_this.chooseClass(d.PARTY))
                .on("click", function(d) {
                                deselectCircles();
                                let csv = "data/Year_Timeline_" + d.YEAR + ".csv";
                                d3.select(this).attr("r", _this.svgHeight / 10 + 5).attr("id", "selected").classed("highlighted", true);
                                d3.csv(csv, function (error, electoralVoteChart) {
                                    _this.electoralVoteChart.update(electoralVoteChart, _this.colorScale);
                                    _this.votePercentageChart.update(electoralVoteChart);
                                    _this.tileChart.update(electoralVoteChart, _this.colorScale);
                            });
                    });
        let deselectCircles = function() {
            let selectedCircles = _this.svg.selectAll("#selected")
                                            .attr("r", _this.svgHeight / 10 )
                                            .attr("id", function(f){ return f.YEAR;})
                                            .classed("highlighted", false);
        }


        //Append text information of each year right below the corresponding circle
        //HINT: Use .yeartext class to style your text elements
        let yearText = _this.svg.selectAll("text").data(_this.electionWinners);
        let text  = yearText.enter().append("text");

        yearText.exit().remove();
        yearText = text.merge(yearText);

        yearText.attr("x", d=> yrScale(d.YEAR) - 20)
                .attr("y", _this.svgHeight / 2 + 30)
                .attr("class", "yearText")
                .text(d=>d.YEAR);



    //******* TODO: EXTRA CREDIT *******

    //Implement brush on the year chart created above.
    //Implement a call back method to handle the brush end event.
    //Call the update method of shiftChart and pass the data corresponding to brush selection.
    //HINT: Use the .brush class to style the brush.

    _this.shiftChart.update("", "year");
    let brushed = function() {
        let selection = d3.event.selection, selectedStates = [], k = 0;
        //console.log(selection);
        if(selection){
            let states = _this.svg.selectAll("circle").attr("cx", function(d){
                let cx = parseFloat(d3.select(this).attr("cx")), r = parseFloat(d3.select(this).attr("r")); 
                if(cx - r - 5 >= selection[0] && cx - r - 5 < selection[1])
                {
                    if((cx + r + 5) <= selection[1])
                        selectedStates[k++] = d3.select(this).data()[0].YEAR;
                }
                return cx;
            });
        }
        _this.shiftChart.update(selectedStates, "year");
    }
    let brush = d3.brushX().extent([[0, _this.svgHeight / 2 + 15],[_this.svgWidth, _this.svgHeight / 2 + 35]]).on("end", brushed);
    _this.svg.append("g").attr("class", "brush").call(brush);
    
    };

};