<?php
// https://code.google.com/p/minify/
// http://getbootstrap.com/
if (extension_loaded("zlib") && (ini_get("output_handler") != "ob_gzhandler")) {
    ini_set("zlib.output_compression", 1);
}
require_once('defines.php');
require_once('content.php');

$curCat = 0;
if($_GET["cat"]) { $curCat = (int)$_GET["cat"];}
if($curCat < 0 || $curCat > $content->getNCategories()-1)
	$curCat = 0;

$curPag = 0;
if($_GET["page"]) { $curPag = (int)$_GET["page"];}
if($curPag < 0 || $curPag > $content->getNPages($curCat)-1)
	$curPag = 0;
?>
<!doctype html>
<html lang="en">
	<head>
		<meta charset="utf-8">
	    <meta http-equiv="X-UA-Compatible" content="IE=edge">
		<meta http-equiv="Content-Type" content="text/html;charset=utf-8" />
    	<meta name="viewport" content="width=device-width, initial-scale=1">
		<title><?php echo $site_title;?></title>
		<meta name="description" content="">
		<meta name="author" content="Jan Beneke <mail@lightgraffiti.de>">
		<meta name="viewport" content="width=device-width, initial-scale=1">
		<link rel="icon" type="image/png" href="img/fav.png" />

		<link href='http://fonts.googleapis.com/css?family=Source+Sans+Pro:400,700,400italic|Raleway:400,700' rel='stylesheet' type='text/css'>
		<!-- <link rel="stylesheet" href="css/lightbox.css"> -->
		<!-- <link rel="stylesheet" type="text/css" href="js/fancybox/jquery.fancybox.css?v=2.1.5" media="screen" /> -->
		<!-- <link rel="stylesheet" type="text/css" href="js/fancybox/helpers/jquery.fancybox-buttons.css?v=1.0.5" /> -->
		<!-- <link rel="stylesheet" type="text/css" href="js/fancybox/helpers/jquery.fancybox-thumbs.css?v=1.0.7" /> -->
		<link href="http://getbootstrap.com/dist/css/bootstrap.min.css" rel="stylesheet">
		<link href="css/style.css" rel="stylesheet">
	</head>

	<body>
	    <nav class="navbar navbar-default navbar-fixed-top">
	      <div class="container">
	        <div class="navbar-header">
	          <button type="button" class="navbar-toggle collapsed" data-toggle="collapse" data-target="#navbar" aria-expanded="false" aria-controls="navbar">
	            <span class="sr-only">Toggle navigation</span>
	            <span class="icon-bar"></span>
	            <span class="icon-bar"></span>
	            <span class="icon-bar"></span>
	          </button>
	          <a class="navbar-brand" href="?cat=0"><span class="glyphicon glyphicon-flash" aria-hidden="true"></span> <?php echo $site_title;?></a>
	        </div>
	        <div id="navbar" class="navbar-collapse collapse">
				<ul class="nav navbar-nav">
					<?php
						foreach($content->getCategories() as $i => $cat) {
							if( $i == 0 || !$content->getCategoryType($i)) {continue;}
							if( $content->getNPages($i) > 1) {
								echo '<li class="dropdown">';
								echo '<a href="#" class="dropdown-toggle" data-toggle="dropdown" role="button" aria-expanded="false">';
								echo '<span class="glyphicon '.$content->getCategoryGlyph($i).'" aria-hidden="true"></span> '.$cat.' <span class="caret"></span></a>';
								echo '<ul class="dropdown-menu" role="menu">';
								foreach($content->getPageTitles($i) as $j => $pag) {
									if( $i == $curCat && $j == $curPag) {
										echo '<li class="active">';
									} else {
										echo '<li>';
									}
									echo '<a href="?cat='.$i.'&page='.$j.'">'.$pag.'</a>';
									echo '</li>';
								}
								echo '</ul></li>';
							} else {
								if( $i == $curCat) {
									echo '<li class="active">';
								} else {
									echo '<li>';
								}
								echo '<a href="?cat='.$i.'"><span class="glyphicon '.$content->getCategoryGlyph($i).'" aria-hidden="true"></span> '.$cat.'</a>';
								echo '</li>';
							}
						}
					?>
				</ul>
				<ul class="nav navbar-nav navbar-right">
					<li class="dropdown">
						<a href="#" class="dropdown-toggle" data-toggle="dropdown" role="button" aria-expanded="false"><span class="glyphicon glyphicon-menu-hamburger" aria-hidden="true"></span> More <span class="caret"></span></a>
						<ul class="dropdown-menu" role="menu">
						    <?php
								foreach($content->getCategories() as $i => $cat) {
									if( $content->getCategoryType($i)) {continue;}
									if( $i == $curCat) {
										echo '<li class="active">';
									} else {
										echo '<li>';
									}
									echo '<a href="?cat='.$i.'"><span class="glyphicon '.$content->getCategoryGlyph($i).'" aria-hidden="true"></span> '.$cat.'</a>';
									echo '</li>';
								}
							?>
						</ul>
					</li>
				</ul>
	        </div>
	      </div>
	    </nav>

	    <?php
	    echo $content->getPageContent($curCat, $curPag);
	    ?>

	      <hr>

	      <footer>
	      <div class="container">
  <div class="row">
    <div class="col-xs-6">
      <p class="copyright"><?php echo $copyright;?></p>
    </div>
    <div class="col-xs-6">
      <div class="pull-right dropup" style="margin-bottom: 20px">
			  <button class="btn btn-info btn-sm dropdown-toggle" type="button" id="dropdownMenu2" data-toggle="dropdown" aria-expanded="true">
			    Follow me on ...
			    <span class="caret"></span>
			  </button>
			  <ul class="dropdown-menu" role="menu" aria-labelledby="dropdownMenu2">
			    <li role="presentation"><a role="menuitem" tabindex="-1" href="http://de.linkedin.com/pub/jan-beneke/44/654/61b"><img class="social" src="content/contact/img/linkedin.png" target="_blank" width="16" align="left">LinkedIn</a></li>
			    <li role="presentation"><a role="menuitem" tabindex="-1" href="https://www.xing.com/profile/Jan_Beneke2"><img class="social" src="content/contact/img/xing.png" target="_blank" width="16" align="left">XING</a></li>
			    <li role="presentation"><a role="menuitem" tabindex="-1" href="https://www.researchgate.net/profile/Jan_Beneke2"><img class="social" src="content/contact/img/rg.png" target="_blank" width="16" align="left">ResearchGate</a></li>
			  </ul>
			</div>
   </div>
  </div>
	        
	        
	      </footer>
	    </div>

		<script type="text/javascript">$("[data-toggle=tooltip]").tooltip();</script>

	    <script src="https://ajax.googleapis.com/ajax/libs/jquery/1.11.2/jquery.min.js"></script>
	    <script src="http://getbootstrap.com/dist/js/bootstrap.min.js"></script>
	    <!-- IE10 viewport hack for Surface/desktop Windows 8 bug -->
	    <script src="http://getbootstrap.com/assets/js/ie10-viewport-bug-workaround.js"></script>
	</body>
</html>