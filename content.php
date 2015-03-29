<?php
require_once("defines.php");

class Content {
    var $json;	// JSON array
    var $cats;	// Categories
    var $catGlyphs;
    var $catType;
    var $pages;	// Pages, sorted by categorie IDs

    function read_json() {
		global $content_path;
		$cJson = file_get_contents($content_path);
		$cArray = json_decode( $cJson, true);
		return $cArray;
	}

	function parse_json() {
		foreach($this->json["categories"] as $val) {
			$this->cats[$val["id"]] = $val["title"];
			$this->catGlyphs[$val["id"]] = $val["glyphicon"];
			$this->catType[$val["id"]] = $val["type"] == "main";
		}
		// var_dump($this->cats);

		foreach($this->json["pages"] as $val) {
			$container = $this->pages[$val["catId"]];
			$container[$val["id"]] = $val;
			$this->pages[$val["catId"]] = $container;
		}
		// var_dump($this->pages);
		// echo "<br/>";
	}

    function Content() {
    	// echo "<p>-</p><p>-</p><p>-</p><p>-</p><p>-</p><p>-</p>";
        $this->json = $this->read_json();
        $this->parse_json();
    }


    function getCategories() {
    	return $this->cats;
    }

    function getNCategories() {
    	return count($this->cats);
    }

    function getCategoryGlyph($i) {
    	return $this->catGlyphs[$i];
    }

    function getCategoryType($i) {
    	return $this->catType[$i];
    }


    function getPages($catId) {
    	return $this->pages[$catId];
    }

    function getNPages($catId) {
    	return count($this->getPages($catId));
    }

    function getPageTitles($catId) {
    	$titles = array();
    	for($i = 0; $i < $this->getNPages($catId); $i++) {
    		$titles[$i] = $this->getPageTitle($catId, $i);
    	}
    	return $titles; 
    }

    function getPageTitle($catId, $id) {
    	return $this->getPages($catId)[$id]["title"]; 
    }

    function getPageContent($catId, $id) {
    	$path = $this->getPages($catId)[$id]["html"];
    	if( $path && !empty($path) && is_file($path)) {
    		return file_get_contents($path);
    	} else {
    		return '<div class="container"><div class="alert alert-danger" role="alert">Page content can not be found.</div></div>';
    	}
    }
}

$content = new Content;

?>