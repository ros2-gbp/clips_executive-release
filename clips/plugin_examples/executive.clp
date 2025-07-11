; Copyright (c) 2024-2025 Carologistics
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

(defrule print-time
  (time ?now)
  =>
  (printout info "time between agenda refresh and rule fire: " (- (now) ?now) crlf)
  (printout green "ROS time: " (now) crlf)
  (printout blue "SYS time: " (now-systime) crlf)
)
