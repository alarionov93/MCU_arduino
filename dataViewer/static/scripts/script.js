
/* TODO: create choosing data source dialogs */
var jsonTrackerData;
function getTrackerData() {
    $.ajax({
        method: 'GET',
        url: '/get_tracker_chg_info/' + $("select#measure").val(),
        dataType: 'json',
        contentType: "application/json",
        success: function (resp) {
            jsonTrackerData = resp;
            // Load the Visualization API and the corechart package.
            google.charts.load('current', {'packages': ['corechart']});
            // Set a callback to run when the Google Visualization API is loaded.
            google.charts.setOnLoadCallback(drawTrackerChgChart);
        }
    });
}
function drawTrackerChgChart() {
    // Create the data table.
    // var data = new google.visualization.DataTable();
    // data.addColumn('string', 'Abc');
    // data.addColumn('number', 'Temp2');
    // data.addRows(jsonData);
    var data = google.visualization.arrayToDataTable(jsonTrackerData);

    // Set chart options
    var options = {'title':'Tracker Chg Info',
                   'width':1000,
                   'height':400};

    // Instantiate and draw our chart, passing in some options.
    var chart = new google.visualization.LineChart(document.getElementById('tracker_chart_div'));
    chart.draw(data, options);
}
$(document).ready(function () {

  $("select#measure").change(function() {
      getData();
      getTrackerData();
  });

  var jsonData;
  function getData() {

      $.ajax({
        method: 'GET',
        url: '/get_measures/' + $("select#measure").val(),
        dataType: 'json',
        contentType:"application/json",
        success: function(resp) {
          jsonData = resp;
          // Load the Visualization API and the corechart package.
          google.charts.load('current', {'packages':['corechart']});
          // Set a callback to run when the Google Visualization API is loaded.
          google.charts.setOnLoadCallback(drawChart);
        }
      });
  }
  getData();

  // Callback that creates and populates a data table,
  // instantiates the pie chart, passes in the data and
  // draws it.
  function drawChart() {
    // Create the data table.
    // var data = new google.visualization.DataTable();
    // data.addColumn('string', 'Abc');
    // data.addColumn('number', 'Temp2');
    // data.addRows(jsonData);
    var data = google.visualization.arrayToDataTable(jsonData);

    // Set chart options
    var options = {'title':'Moto Data Visualizer',
                   'width':1200,
                   'height':700};

    // Instantiate and draw our chart, passing in some options.
    var chart = new google.visualization.LineChart(document.getElementById('main_chart_div'));
    chart.draw(data, options);
  }
});