(* ::Package:: *)

(*************************************************************************)
(*                                 cool.m                                *)
(*************************************************************************)
(*                                                                       *)
(* Author: Renato Maia Matarazzo Orsino                                  *)
(* 2020-04-29                                                            *)
(*                                                                       *)
(*************************************************************************)



hexToRGB = RGBColor @@ (IntegerDigits[ToExpression @ StringReplace[#, "#"->"16^^"], 256, 3]/255.)&;

Cool = hexToRGB/@{
	"#2850ad",
	"#fccc0a",
	"#ee352e",
	"#00add0",
	"#808183",
	"#00933c",
	"#b933ad",
	"#ff6319",
	"#6cbe45",
	"#a7a9ac",
	"#996633",
	"#272727"
	};


SetOptions[ListPointPlot3D, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 16}, 
  PlotStyle -> Cool
  ];

SetOptions[Plot, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}, 
  PlotStyle -> Cool
  ];
  
SetOptions[LogPlot, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}, 
  PlotStyle -> Cool
  ];
  
SetOptions[Plot3D, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 16}, 
  PlotStyle -> Cool
  ];
  
SetOptions[ParametricPlot, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}, 
  PlotStyle -> Cool
  ];
  
SetOptions[ParametricPlot3D, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}, 
  PlotStyle -> Cool
  ];
  
SetOptions[ListPlot, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}, 
  PlotStyle -> Cool
  ];
  
SetOptions[ContourPlot, 
  BaseStyle -> {FontFamily -> "Roboto", FontSize -> 20}
  ];  
   	
