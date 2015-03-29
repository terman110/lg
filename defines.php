<?php

$site_title = 'lightgraffiti';
$short_title = 'lg';
$site_mail = "contact@janbeneke.de";	
$site_url = "localhost/lg"; //"http://www.lightgraffiti.de/lg";

$content_path = "content.json";

date_default_timezone_set('Europe/Berlin');
$copyright = "&copy; Jan Beneke ".date("Y");

function mailer( $to, $subject, $message) {
	$headers = 'From: ' . $site_mail . "\r\n" .
	    'Reply-To: ' . $site_mail . "\r\n" .
	    'X-Mailer: PHP/' . phpversion();
	mail($to, $subject, $message, $headers);
}
?>