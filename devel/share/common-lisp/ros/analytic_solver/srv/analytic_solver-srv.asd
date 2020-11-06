
(cl:in-package :asdf)

(defsystem "analytic_solver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "analytic" :depends-on ("_package_analytic"))
    (:file "_package_analytic" :depends-on ("_package"))
  ))