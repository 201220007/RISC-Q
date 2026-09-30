# STEPS.ROUTE_DESIGN.TCL.POST (inc/impl-settings.tcl), P3c. route_design -directive AggressiveExplore can end
# with a few nets unrouted or in conflict: its in-route physical optimisation re-places cells late, and the
# nets of the moved cells are not routed again. The post-route phys_opt then skips ("design is not fully
# routed") and every timing number carries estimated net delays (P3c trials 3 and 4). A plain route_design
# on the partly routed design routes only what is left; it is a no-op when the design is fully routed.
proc riscq_route_counts {} {
  set rs [report_route_status -return_string]
  set unr 0; set err 0
  regexp {of unrouted nets[.]*\s*:\s*([0-9]+)} $rs -> unr
  regexp {nets with routing errors[.]*\s*:\s*([0-9]+)} $rs -> err
  return [list $unr $err]
}
lassign [riscq_route_counts] _unr _err
puts "\[route-finish\] after route_design: $_unr unrouted net(s), $_err net(s) with routing errors"
for {set _pass 1} {($_unr || $_err) && $_pass <= 2} {incr _pass} {
  route_design
  lassign [riscq_route_counts] _unr _err
  puts "\[route-finish\] pass $_pass: $_unr unrouted net(s), $_err net(s) with routing errors"
}
if {$_unr || $_err} { puts "\[route-finish\] WARNING: the design is still not fully routed" }
